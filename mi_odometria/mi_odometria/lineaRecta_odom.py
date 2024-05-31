import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

class LineaRectaOdom(Node):

    # Posición inicial (m)
    _init_x = None
    _init_y = None
    # Posición actual (m)
    _odom_x = None
    _odom_y = None

    def __init__(self,
                 distancia = 1.0,
                 periodo = 0.1,
                 velocidad_maxima = 0.1,
                 velocidad_minima = 0.01,
                 regulador_p = 1.5):
        # Inicialización de variables
        # Distancia objetivo (m)
        self._distancia_objetivo = distancia
        # Periodo de actualización (s)
        self._tiempo_periodo = periodo
        # Velocidad (m/s) y ganancia del regulador P
        self._velocidad_maxima = velocidad_maxima
        self._velocidad_minima = velocidad_minima
        self._regulador_p = regulador_p

        # Crea el nodo
        super().__init__('lineaRecta_odom')
        # Crea el publisher para el control de velocidad
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Crea el subscriber para el posicionamiento por odometría
        self.subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        # Crea el timer para el bucle principal
        self.mainTimer = self.create_timer(self._tiempo_periodo, self.timer_callback)
        self.get_logger().info('Nodo lineaRecta_odom creado')
        # Comunicación con el usuario
        self.get_logger().info('''Movimiento lineal iniciado.\n
                        Modo de control: odometría.\n
                        Distancia solicitada: {distancia} m\n
                        Velocidad de movimiento: {velocidad} m/s'''.format(
                          distancia = self._distancia_objetivo,
                          velocidad = self._velocidad_minima
                      ))

    def timer_callback(self):
        msg = Twist()
        # Si aún no se conoce la posición del robot, termina la llamada
        if self._odom_x == None or self._odom_y == None:
            return
        # Calcula la distancia recorrida actualmente
        delta_x = self._odom_x-self._init_x
        delta_y = self._odom_y-self._init_y
        distancia_actual = math.sqrt(math.pow(delta_x,2)+math.pow(delta_y,2))
        # Información al usuario sobre el estado actual del movimiento
        self.get_logger().info('{current}m/{goal}m ({percent}%)'.format(
            goal = round(self._distancia_objetivo, 2),
            current = round(distancia_actual, 2),
            percent = round(100*distancia_actual/self._distancia_objetivo,2)
        ))
        # Envía comandos de avance mientras la distancia objetivo no se haya superado
        if distancia_actual < self._distancia_objetivo:
            msg.linear.x = max(min(self._regulador_p*(self._distancia_objetivo-distancia_actual), self._velocidad_maxima), self._velocidad_minima)
        # Envía comandos de parada cuando se supera el tiempo final
        self.publisher.publish(msg)

    def odom_callback(self, msg: Odometry):
        # Si aún no se ha establecido, registra la posición inicial
        if self._init_x == None or self._init_y == None:
            self._init_x = msg.pose.pose.position.x
            self._init_y = msg.pose.pose.position.y
        # Actualiza la posición actual del robot sobre el plano xy
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y

def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    lineaRecta_odom = LineaRectaOdom()
    # Añade el nodo a la lista de tareasa
    rclpy.spin(lineaRecta_odom)
    # Destruye el nodo
    lineaRecta_odom.destroy_node()
    # Finaliza la ejecución
    rclpy.shutdown()

if __name__ == '__main__':
    main()