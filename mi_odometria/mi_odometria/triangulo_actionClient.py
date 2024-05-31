import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from mi_odometria_interfaces.action import Triangulo

class Triangulo_ActionClient(Node):

    def __init__(self, tipo = 0, distancia = 1.0):
        # Crea el nodo
        super().__init__('triangulo_actionClient')
        # Crea el cliente de servicio y espera a conectarse
        self.client = ActionClient(self, Triangulo, 'triangulo')

    def enviar_peticion(self, longitud_lado = 1):
        msg = Triangulo.Goal()
        # Define el contenido de la petición
        msg.longitud_lado = longitud_lado
        # Conexión al servidor
        self.get_logger().info('Conectando con el servidor...')
        self.client.wait_for_server()
        # Envío de la petición
        self.get_logger().info('Servidor conectado. Enviando petición.')
        self.action_future = self.client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self.action_future.add_done_callback(self.estado_peticion_callback)
        return
    
    def estado_peticion_callback(self, future):
        # COmprueba si la petición ha sido aceptada o rechazada
        action_handle = future.result()
        if not action_handle.accepted:
            self.get_logger().info('Petición rechazada')
            return
        self.get_logger().info('Petición aceptada')
        # Añade el callback de resultado a la acción
        self.resultado_future = action_handle.get_result_async()
        self.resultado_future.add_done_callback(self.resultado_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        tipo_movimiento = feedback.tipo_movimiento
        id_movimiento = feedback.id_movimiento
        estado_movimiento = feedback.estado_movimiento
        if tipo_movimiento == 'Lineal':
            self.get_logger().info('Lado actual: {}\n Distancia actual: {}'.format(id_movimiento,estado_movimiento))
        elif tipo_movimiento == 'Rotación':
            self.get_logger().info('Vértice actual: {}\n Ángulo de rotación restante: {}'.format(id_movimiento,estado_movimiento))
        
    def resultado_callback(self, future):
        # Recibe el resultado de la acción
        result = future.result().result
        self.get_logger().info('El servidor ha finalizado la acción. Resultado: {}'.format(result.exito))
        rclpy.shutdown()
    
def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    triangulo_actionClient = Triangulo_ActionClient()
    # Solicita los datos del movimiento deseado al usuario
    longitud_lado = float(input('Longitud del lado del triángulo:\n'))
    # Envía una petición al servidor desde el cliente
    triangulo_actionClient.enviar_peticion(longitud_lado)
    # Mantiene el cliente en ejecución
    rclpy.spin(triangulo_actionClient)

if __name__ == '__main__':
    main()