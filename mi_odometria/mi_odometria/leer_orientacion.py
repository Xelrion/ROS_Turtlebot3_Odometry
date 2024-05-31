import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class LeerOrientacion(Node):
    def __init__(self):
        super().__init__('LeerOrientacion')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom', # The topic to subscribe to
            self.listener_callback,
            10 # Queue size
        )
        self.subscription
        
    def listener_callback(self, msg):
        # Obtenemos el cuaternio
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        # Convierte angulos de euler(roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        # Muestra los angulos de euler
        self.get_logger().info(f'Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')

def main(args=None):
    rclpy.init(args=args)
    leer_orientacion=LeerOrientacion()
    rclpy.spin(leer_orientacion)
    leer_orientacion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()