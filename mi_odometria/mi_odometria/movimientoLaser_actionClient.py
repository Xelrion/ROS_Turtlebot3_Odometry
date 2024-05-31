import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from mi_odometria_interfaces.action import MovLinRot

class Movimiento_ActionClient(Node):

    def __init__(self):
        # Crea el nodo
        super().__init__('movimiento_actionClient')
        # Crea el cliente de acción y espera a conectarse
        self.client = ActionClient(self, MovLinRot, 'movimiento')

    def enviar_peticion(self, tipo_movimiento = 0, magnitud_movimiento = 1.0):
        msg = MovLinRot.Goal()
        # Define el contenido de la petición
        msg.tipo = tipo_movimiento
        msg.magnitud = magnitud_movimiento
        # Conexión al servidor
        self.get_logger().info('Conectando con el servidor...')
        self.client.wait_for_server()
        # Envío de la petición
        self.get_logger().info('Servidor conectado. Enviando petición.')
        self.action_future = self.client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self.action_future.add_done_callback(self.estado_peticion_callback)
        return
    
    def estado_peticion_callback(self, future):
        # Comprueba si la petición ha sido aceptada o rechazada
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
        estado_movimiento = feedback.estado_movimiento
        if tipo_movimiento == 'Lineal':
            self.get_logger().info('Distancia actual: {}'.format(estado_movimiento))
        elif tipo_movimiento == 'Rotación':
            self.get_logger().info('Ángulo de rotación restante: {}'.format(estado_movimiento))
        
    def resultado_callback(self, future):
        # Recibe el resultado de la acción
        result = future.result().result
        self.get_logger().info('El servidor ha finalizado la acción. Resultado: {}'.format(result.exito))
        rclpy.shutdown()
    
def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    movimiento_actionClient = Movimiento_ActionClient()
    # Solicita los datos del movimiento deseado al usuario
    tipo = int(input('Tipo de movimiento deseado: 0 para lineal, 1 para rotación\n'))
    magnitud = float(input('Magnitud del movimiento deseado: metros en lineal, ángulos sexagesimales en rotación\n'))
    # Envía una petición al servidor desde el cliente
    movimiento_actionClient.enviar_peticion(tipo, magnitud)
    # Mantiene el cliente en ejecución
    rclpy.spin(movimiento_actionClient)

if __name__ == '__main__':
    main()