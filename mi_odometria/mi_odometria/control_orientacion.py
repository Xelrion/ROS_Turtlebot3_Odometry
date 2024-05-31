import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations as tft
from math import pi

class ControlOrientacion(Node):
    def __init__(self):
        super().__init__('turn_to_orientation')
        # Publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odom', # Topic to get robot's current state
            self.odometry_callback,
            10 # Queue size
        )
        # orientacion deseada en radianes
        self.desired_orientation = pi / 2
        # Ganancia proporcional para el giro
        self.p_gain = 1.0 # You can tune this value
    
    def odometry_callback(self, msg):
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, current_yaw = tft.euler_from_quaternion(quaternion)
        # Calculamos el error
        angular_error = self.wrap_angle(self.desired_orientation - current_yaw)
        # Control proporcional para determinar la velocidad de giro
        angular_velocity = self.p_gain * angular_error
        # Publicamos esa vlocidad
        twist = Twist()
        twist.angular.z = angular_velocity
        twist.linear.x = 0.0 # Ensure linear motion is zero
        self.publisher.publish(twist)
        # Log
        self.get_logger().info(f'Error: {angular_error:.2f}, Commanded Angular Velocity: {angular_velocity:.2f}')
    
    def wrap_angle(self, angle):
        """
        Nos aseguramos que alngulo este entre [-pi, pi]
        """
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ControlOrientacion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()