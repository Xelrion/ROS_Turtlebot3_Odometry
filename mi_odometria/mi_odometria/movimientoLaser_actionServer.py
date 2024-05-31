import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer

from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mi_odometria_interfaces.action import MovLinRot

import math

class Movimiento_ActionServer(Node):

    # Posición inicial (m)
    _init_x = None
    _init_y = None
    _init_yaw = None
    # Posición actual (m)
    _odom_x = None
    _odom_y = None
    _odom_yaw = None
    # Detección de obstáculos
    _path_free = True
    # Variable para el seguimiento de la ejecución correcta de la petición
    _success = True
    
    def __init__(self,
                 periodo = 0.1,
                 velocidad_lineal_maxima = 0.1,
                 velocidad_lineal_minima = 0.01,
                 velocidad_rotacion_maxima = 0.5,
                 velocidad_rotacion_minima = 0.1,
                 regulador_p = 1.5,
                 max_dif_angulo = 2.5,
                 max_security_distance = 0.25,
                 laser_angles_detection = 10):
        # Inicialización de variables
        # Periodo de actualización (s)
        self._tiempo_periodo = periodo
        # Velocidad (m/s) y ganancia del regulador P
        self._velocidad_lineal_maxima = velocidad_lineal_maxima
        self._velocidad_lineal_minima = velocidad_lineal_minima
        self._velocidad_rotacion_maxima = velocidad_rotacion_maxima
        self._velocidad_rotacion_minima = velocidad_rotacion_minima
        self._regulador_p = regulador_p
        # Tolerancia de error de rotación
        self._max_dif_angulo = math.radians(max_dif_angulo)
        # Margen de ángulos y distancia de detección de obstáculos
        self._max_security_distance = max_security_distance
        self._laser_angles_detection = laser_angles_detection

        # Crea el nodo
        super().__init__('movimiento_actionServer')
        # Crea el grupo de callbacks recurrente
        self.reentrant_group = ReentrantCallbackGroup()
        # Crea el publisher para el control de velocidad
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.reentrant_group)
        # Crea el subscriber para el posicionamiento por odometría
        self.subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=self.reentrant_group)
        # Crea el subscriber para la detección de obstáculos mediante láser
        self.subLaser = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10, callback_group=self.reentrant_group)
        # Crea el servidor de servicio
        self.action_server = ActionServer(self, MovLinRot, 'movimiento', self.action_callback, callback_group=self.reentrant_group)
        # Establece la frecuencia de actualización del nodo
        self.loop_rate = self.create_rate((1/self._tiempo_periodo), self.get_clock())

    def action_callback(self, action_handle):
        respuesta = MovLinRot.Result()
        # Lee la petición
        tipo = action_handle.request.tipo
        magnitud = action_handle.request.magnitud
        # Borra la posición de inicio actual
        self._init_x = None
        self._init_y = None
        self._init_yaw = None
        # Espera a que la odometría establezca una posición de inicio para el robot
        while(self._init_x == None or self._init_y == None or self._init_yaw == None):
            self.loop_rate.sleep()
        # Inicia el tipo de movimiento pedido por el cliente
        if tipo == 0:
            self.get_logger().info('''Petición de movimiento recibida..\n
                        Tipo de movimiento: lineal.\n
                        Distancia solicitada: {distancia} m\n
                        Velocidad de movimiento: {velocidad} m/s'''.format(
                          distancia = magnitud,
                          velocidad = self._velocidad_lineal_maxima
                      ))
            self.mov_lineal(action_handle, magnitud)
        elif tipo == 1:
            self.get_logger().info('''Petición de movimiento recibida..\n
                        Tipo de movimiento: rotación.\n
                        Giro solicitado: {angulo} grados\n
                        Velocidad de movimiento: {velocidad} m/s'''.format(
                          angulo = magnitud,
                          velocidad = self._velocidad_rotacion_maxima
                      ))
            self.mov_rotacion(action_handle, magnitud)
        # Devuelve una respuesta al cliente
        self.get_logger().info('Movimiento finalizado')
        action_handle.succeed()
        respuesta.exito = True
        return respuesta
    
    def mov_lineal(self, action_handle, distancia):
        # Crea el mensaje de velocidad
        self.get_logger().info('Iniciando movimiento: avance lineal. Distancia objetivo: {}'.format(distancia))
        msg_vel = Twist()
        # Ejecuta el bucle principal de movimiento hasta que se supere la distancia objetivo
        distancia_actual = self.distancia_actual()
        while(distancia_actual < distancia):
            # El robot solo se desplaza si es seguro continuar. En caso contrario, se detiene.
            if self._path_free:
                msg_vel.linear.x = max(min(self._regulador_p*(distancia-distancia_actual), self._velocidad_lineal_maxima), self._velocidad_lineal_minima)
            else:
                msg_vel.linear.x = 0.0
            self.enviar_feedback(action_handle, 'Lineal', distancia_actual)
            self.publisher.publish(msg_vel)
            # Información al usuario sobre el estado actual del movimiento
            self.get_logger().info('{current}m/{goal}m ({percent}%)'.format(
                goal = round(distancia, 2),
                current = round(distancia_actual, 2),
                percent = round(100*distancia_actual/distancia,2)
            ))
            self.loop_rate.sleep()
            distancia_actual = self.distancia_actual()
        # Detiene el robot y envía un mensaje de respuesta al cliente
        msg_vel.linear.x = 0.0
        self.publisher.publish(msg_vel)
        return
    
    def mov_rotacion(self, action_handle, rotacion):
        # Establece el mensaje de movimiento con la velocidad rotacional máxima, según la dirección de giro
        self.get_logger().info('Iniciando movimiento: rotación. Ángulo de giro: {}'.format(rotacion))
        msg_rot = Twist()
        # Variables de seguimiento de orientación inicial y objetivo
        angulo_restante = math.radians(rotacion)
        angulo_objetivo = self._init_yaw + angulo_restante
        # Se ajusta el ángulo objetivo, ya que la odometría va de -180º a 180º
        if angulo_objetivo > math.radians(180):
            angulo_objetivo -= math.radians(360)
        # Envía el mensaje hasta que el robot alcance la orientación objetivo
        while abs(angulo_restante) > self._max_dif_angulo:
            self.enviar_feedback(action_handle, 'Rotación', math.degrees(angulo_restante))
            msg_rot.angular.z = msg_rot.angular.z = math.copysign(1.0, rotacion) * max(min(self._regulador_p*(abs(angulo_restante)), self._velocidad_rotacion_maxima), self._velocidad_rotacion_minima)
            self.publisher.publish(msg_rot)
            # Información al usuario sobre el estado actual del movimiento
            self.get_logger().info('{current}º/{goal}º --- Ángulo restante: ({remaining}º)'.format(
                goal = round(math.degrees(angulo_objetivo), 2),
                current = round(math.degrees(self._odom_yaw), 2),
                remaining = round(math.degrees(angulo_restante),2)
            ))
            self.loop_rate.sleep()
            # Actualiza el ángulo de rotación
            angulo_restante = angulo_objetivo - self._odom_yaw
        # Detiene el robot al terminar
        msg_rot.angular.z = 0.0
        self.publisher.publish(msg_rot)
        self.get_logger().info('Movimiento finalizado: rotación.')
        return
    
    def enviar_feedback(self, action_handle, tipo_movimiento, estado_movimiento):
        feedback = MovLinRot.Feedback()
        feedback.tipo_movimiento = tipo_movimiento
        feedback.estado_movimiento = estado_movimiento
        action_handle.publish_feedback(feedback)

    def odom_callback(self, msg: Odometry):
        # Extrae el cuaternio de orientación y la orientación actual
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        # Si aún no se ha establecido, registra la posición inicial
        if self._init_x == None or self._init_y == None or self._init_yaw == None:
            self._init_x = msg.pose.pose.position.x
            self._init_y = msg.pose.pose.position.y
            self._init_yaw = yaw
        # Actualiza la posición actual del robot sobre el plano xy
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = yaw

    def distancia_actual(self):
        # Calcula la distancia recorrida actualmente
        delta_x = self._odom_x-self._init_x
        delta_y = self._odom_y-self._init_y
        distancia_actual = math.sqrt(math.pow(delta_x,2)+math.pow(delta_y,2))
        return distancia_actual
    
    def laser_callback(self, msg:LaserScan):
        # Comprueba si hay un obstáculo frente al robot en el rango de ángulos solicitado frente a él
        angle = 0
        laser_measurement = 0
        obstacle_detected = False
        while angle < self._laser_angles_detection:
            if msg.ranges[0-laser_measurement] < self._max_security_distance or msg.ranges[0+laser_measurement] < self._max_security_distance:
                obstacle_detected = True
                break
            angle += math.degrees(msg.angle_increment)
            laser_measurement += 1
        # Si se ha detectado obstáculo, se detiene el robot; si no, se reanuda la marcha
        if obstacle_detected and self._path_free == True:
            self._path_free = False
            self.get_logger().info('''Se ha detectado un obstáculo en el camino. El movimiento se detendrá hasta que sea seguro continuar.''')
        elif not obstacle_detected and self._path_free == False:
            self._path_free = True
            self.get_logger().info('''El obstáculo ya no está presente. Ahora, es seguro continuar.''')

def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    movimiento_actionServer = Movimiento_ActionServer()
    # Crea el executor multihilo y le añade el nodo
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(movimiento_actionServer)
    # Inicia el executor
    executor.spin()
    # Destruye el nodo
    movimiento_actionServer.destroy_node()
    # Finaliza la ejecución
    rclpy.shutdown()

if __name__ == '__main__':
    main()