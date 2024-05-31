import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mi_odometria_interfaces.srv import MovLinRot

import math

class Movimiento_ServiceServer(Node):

    # Posición inicial (m)
    _init_x = None
    _init_y = None
    _init_yaw = None
    # Posición actual (m)
    _odom_x = None
    _odom_y = None
    _odom_yaw = None
    
    def __init__(self,
                 periodo = 0.1,
                 velocidad_lineal_maxima = 0.1,
                 velocidad_lineal_minima = 0.01,
                 velocidad_rotacion_maxima = 0.5,
                 velocidad_rotacion_minima = 0.1,
                 regulador_p = 1.5,
                 max_dif_angulo = 2.5):
        # Inicialización de variables
        # Periodo de actualización (s)
        self._tiempo_periodo = periodo
        # Velocidad (m/s) y ganancia del regulador P
        self._velocidad_lineal_maxima = velocidad_lineal_maxima
        self._velocidad_lineal_minima = velocidad_lineal_minima
        self._velocidad_rotacion_maxima = velocidad_rotacion_maxima
        self._velocidad_rotacion_minima = velocidad_rotacion_minima
        self._regulador_p = regulador_p
        self._max_dif_angulo = math.radians(max_dif_angulo)

        # Crea el nodo
        super().__init__('movimiento_serviceServer')
        # Crea el grupo de callbacks recurrente
        self.reentrant_group = ReentrantCallbackGroup()
        # Crea el publisher para el control de velocidad
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.reentrant_group)
        # Crea el subscriber para el posicionamiento por odometría
        self.subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=self.reentrant_group)
        # Crea el servidor de servicio
        self.service = self.create_service(MovLinRot, 'movimiento', self.service_callback, callback_group=self.reentrant_group)
        # Establece la frecuencia de actualización del nodo
        self.loop_rate = self.create_rate((1/self._tiempo_periodo), self.get_clock())

    def service_callback(self, peticion, respuesta):
        # Borra la posición de inicio actual
        self._init_x = None
        self._init_y = None
        self._init_yaw = None
        # Espera a que la odometría establezca una posición de inicio para el robot
        while(self._init_x == None or self._init_y == None or self._init_yaw == None):
            self.loop_rate.sleep()
        # Inicia el tipo de movimiento pedido por el cliente
        if peticion.tipo == 0:
            self.get_logger().info('''Petición de movimiento recibida..\n
                        Tipo de movimiento: lineal.\n
                        Distancia solicitada: {distancia} m\n
                        Velocidad de movimiento: {velocidad} m/s'''.format(
                          distancia = peticion.magnitud,
                          velocidad = self._velocidad_lineal_maxima
                      ))
            self.mov_lineal(peticion.magnitud)
        elif peticion.tipo == 1:
            self.get_logger().info('''Petición de movimiento recibida..\n
                        Tipo de movimiento: rotación.\n
                        Giro solicitado: {angulo} grados\n
                        Velocidad de movimiento: {velocidad} m/s'''.format(
                          angulo = peticion.magnitud,
                          velocidad = self._velocidad_rotacion_maxima
                      ))
            self.mov_rotacion(peticion.magnitud)
        # Devuelve una respuesta al cliente
        self.get_logger().info('Movimiento finalizado')
        respuesta.exito = True
        return respuesta
    
    def mov_lineal(self, distancia):
        # Crea el mensaje de velocidad
        self.get_logger().info('Iniciando movimiento: avance lineal. Distancia objetivo: {}'.format(distancia))
        msg_vel = Twist()
        distancia_actual = self.distancia_actual()
        # Ejecuta el bucle principal de movimiento hasta que se supere la distancia objetivo
        while(distancia_actual < distancia):
            msg_vel.linear.x = max(min(self._regulador_p*(distancia-distancia_actual), self._velocidad_lineal_maxima), self._velocidad_lineal_minima)
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
    
    def mov_rotacion(self, rotacion):
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
            msg_rot.angular.z = math.copysign(1.0, rotacion) * max(min(self._regulador_p*(abs(angulo_restante)), self._velocidad_rotacion_maxima), self._velocidad_rotacion_minima)
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

def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    movimiento_serviceServer = Movimiento_ServiceServer()
    # Crea el executor multihilo y le añade el nodo
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(movimiento_serviceServer)
    # Inicia el executor
    executor.spin()
    # Destruye el nodo
    movimiento_serviceServer.destroy_node()
    # Finaliza la ejecución
    rclpy.shutdown()

if __name__ == '__main__':
    main()