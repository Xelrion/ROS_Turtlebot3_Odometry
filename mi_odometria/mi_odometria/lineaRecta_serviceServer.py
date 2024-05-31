import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mi_odometria_interfaces.srv import MovLinRot

import math

class LineaRecta_ServiceServer(Node):

    # Distancia recorrida (m)
    _distancia_actual = 0.0
    # Posición inicial (m)
    _init_x = None
    _init_y = None
    # Posición actual (m)
    _odom_x = None
    _odom_y = None
    
    def __init__(self,
                 periodo = 0.1,
                 velocidad_maxima = 0.1,
                 velocidad_minima = 0.01,
                 regulador_p = 1.5):
        # Inicialización de variables
        # Periodo de actualización (s)
        self._tiempo_periodo = periodo
        # Velocidad (m/s) y ganancia del regulador P
        self._velocidad_maxima = velocidad_maxima
        self._velocidad_minima = velocidad_minima
        self._regulador_p = regulador_p

        # Crea el nodo
        super().__init__('lineaRecta_serviceServer')
        # Crea el grupo de callbacks recurrente
        self.reentrant_group = ReentrantCallbackGroup()
        # Crea el publisher para el control de velocidad
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10, callback_group=self.reentrant_group)
        # Crea el subscriber para el posicionamiento por odometría
        self.subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10, callback_group=self.reentrant_group)
        # Crea el servidor de servicio
        self.service = self.create_service(MovLinRot, 'linea_recta', self.service_callback, callback_group=self.reentrant_group)
        # Establece la frecuencia de actualización del nodo
        self.loop_rate = self.create_rate((1/self._tiempo_periodo), self.get_clock())

    def service_callback(self, peticion, respuesta):
        # Comunicación con el usuario
        self.get_logger().info('''Movimiento lineal iniciado.\n
                        Modo de control: odometría.\n
                        Distancia solicitada: {distancia} m\n
                        Velocidad de movimiento: {velocidad} m/s'''.format(
                          distancia = peticion.magnitud,
                          velocidad = self._velocidad_maxima
                      ))
        # Crea el mensaje de velocidad
        msg_vel = Twist()
        # Lee la distancia objetivo especificada en el mensaje
        distancia_objetivo = peticion.magnitud
        # Borra la posición de inicio actual
        self._init_x = None
        self._init_y = None
        # Espera a que la odometría establezca una posición de inicio para el robot
        while(self._init_x == None or self._init_y == None):
            self.loop_rate.sleep()
        # Ejecuta el bucle principal de movimiento hasta que se supere la distancia objetivo
        while(self._distancia_actual < distancia_objetivo):
            msg_vel.linear.x = max(min(self._regulador_p*(distancia_objetivo-self._distancia_actual), self._velocidad_maxima), self._velocidad_minima)
            self.publisher.publish(msg_vel)
            # Información al usuario sobre el estado actual del movimiento
            self.get_logger().info('{current}m/{goal}m ({percent}%)'.format(
                goal = round(distancia_objetivo, 2),
                current = round(self._distancia_actual, 2),
                percent = round(100*self._distancia_actual/distancia_objetivo,2)
            ))
            self.loop_rate.sleep()
        # Detiene el robot y envía un mensaje de respuesta al cliente
        msg_vel.linear.x = 0.0
        self.publisher.publish(msg_vel)
        # Devuelve una respuesta al cliente
        self.get_logger().info('Movimiento finalizado')
        respuesta.exito = True
        return respuesta

    def odom_callback(self, msg: Odometry):
        # Si aún no se ha establecido, registra la posición inicial
        if self._init_x == None or self._init_y == None:
            self._init_x = msg.pose.pose.position.x
            self._init_y = msg.pose.pose.position.y
        # Actualiza la posición actual del robot sobre el plano xy
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        # Calcula la distancia recorrida actualmente
        delta_x = self._odom_x-self._init_x
        delta_y = self._odom_y-self._init_y
        self._distancia_actual = math.sqrt(math.pow(delta_x,2)+math.pow(delta_y,2))
        

def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    lineaRecta_serviceServer = LineaRecta_ServiceServer()
    # Crea el executor multihilo y le añade el nodo
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(lineaRecta_serviceServer)
    # Inicia el executor
    executor.spin()
    # Destruye el nodo
    lineaRecta_serviceServer.destroy_node()
    # Finaliza la ejecución
    rclpy.shutdown()

if __name__ == '__main__':
    main()