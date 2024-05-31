import rclpy
from rclpy.node import Node

from mi_odometria_interfaces.srv import MovLinRot

class LineaRecta_ServiceClient(Node):

    def __init__(self, distancia = 1.0):
        # Inicialización de variables
        self.distancia_objetivo = distancia

        # Crea el nodo
        super().__init__('lineaRecta_serviceClient')
        # Crea el cliente de servicio y espera a conectarse
        self.client = self.create_client(MovLinRot, 'linea_recta')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, intentando de nuevo...')
        self.get_logger().info('Servidor conectado')

    def enviar_peticion(self):
        msg = MovLinRot.Request()
        # Define el contenido de la petición
        msg.magnitud = self.distancia_objetivo
        # Envía la petición y espera a que el servidor la realice
        self.future = self.client.call_async(msg)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    lineaRecta_serviceClient = LineaRecta_ServiceClient()
    # Envía una petición al servidor desde el cliente
    respuesta = lineaRecta_serviceClient.enviar_peticion()
    exito = respuesta.exito
    # Destruye el nodo
    lineaRecta_serviceClient.get_logger().info('El servidor ha concluido la tarea. Respuesta del servidor: {}'.format(exito))
    # Finaliza la ejecución
    lineaRecta_serviceClient.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()