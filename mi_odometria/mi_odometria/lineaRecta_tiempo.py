import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class LineaRectaTiempo(Node):

    # Contador de tiempo (s)
    _tiempo_actual = 0

    def __init__(self,
                 distancia = 1.0,
                 periodo = 0.1,
                 velocidad = 0.05):
        # Inicialización de variables
        self._distancia = distancia
        self._tiempo_final = distancia/velocidad
        self._tiempo_periodo = periodo
        self._velocidad = velocidad

        # Crea el nodo
        super().__init__('lineaRecta_tiempo')
        # Crea el publisher para el control de velocidad
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Crea el timer de actualización para el bucle principal
        self.mainTimer = self.create_timer(self._tiempo_periodo, self.timer_callback)
        self.get_logger().info('Nodo lineaRecta_tiempo creado')
        self.get_logger().info('''Movimiento lineal iniciado.\n
                      Modo de control: tiempo.\n
                      Distancia solicitada: {distancia} m\n
                      Velocidad de movimiento: {velocidad} m/s'''.format(
                          distancia = self._distancia,
                          velocidad = self._velocidad
                      ))

    def timer_callback(self):
        msg = Twist()
        self.get_logger().info('{current}s/{goal}s ({percent}%)'.format(
                goal = round(self._tiempo_final, 2),
                current = round(self._tiempo_actual, 2),
                percent = round(100*(self._tiempo_actual)/(self._tiempo_final),2)
            ))
        # Envía comandos de avance mientras el tiempo no se haya alcanzado
        if self._tiempo_actual < self._tiempo_final:
            msg.linear.x = self._velocidad
            self._tiempo_actual += self._tiempo_periodo
        # Envía comandos de parada cuando se supera el tiempo final
        self.publisher.publish(msg)

def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    lineaRecta_tiempo = LineaRectaTiempo()
    # Añade el nodo a la lista de tareasa
    rclpy.spin(lineaRecta_tiempo)
    # Destruye el nodo
    lineaRecta_tiempo.destroy_node()
    # Finaliza la ejecución
    rclpy.shutdown()

if __name__ == '__main__':
    main()