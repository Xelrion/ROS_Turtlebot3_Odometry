import rclpy
from rclpy.node import Node

from mi_odometria_interfaces.srv import MovLinRot

class Movimiento_ServiceClient(Node):

    def __init__(self, tipo = 0, distancia = 1.0):
        # Crea el nodo
        super().__init__('movimiento_serviceClient')
        # Crea el cliente de servicio y espera a conectarse
        self.client = self.create_client(MovLinRot, 'movimiento')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio no disponible, intentando de nuevo...')
        self.get_logger().info('Servidor conectado')

    def enviar_peticion(self, tipo_movimiento = 0, magnitud_movimiento = 1.0):
        msg = MovLinRot.Request()
        # Define el contenido de la petición
        msg.tipo = tipo_movimiento
        msg.magnitud = magnitud_movimiento
        # Envía la petición y espera a que el servidor la realice
        self.future = self.client.call_async(msg)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main(args=None):
    # Inicia la ejecución
    rclpy.init(args=args)
    # Crea el nodo
    movimiento_serviceClient = Movimiento_ServiceClient()
    # Solicita los datos del movimiento deseado al usuario
    tipo = int(input('Tipo de movimiento deseado: 0 para lineal, 1 para rotación\n'))
    magnitud = float(input('Magnitud del movimiento deseado: metros en lineal, ángulos sexagesimales en rotación\n'))
    # Envía una petición al servidor desde el cliente
    respuesta = movimiento_serviceClient.enviar_peticion(tipo, magnitud)
    exito = respuesta.exito
    # Destruye el nodo
    movimiento_serviceClient.get_logger().info('El servidor ha concluido la tarea. Respuesta del servidor: {}'.format(exito))
    # Finaliza la ejecución
    movimiento_serviceClient.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()