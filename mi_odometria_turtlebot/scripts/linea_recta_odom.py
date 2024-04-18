#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# Velocidad lineal máxima (en m/s)
max_linear_speed = 0.05
# Distancia a avanzar (en metros)
distance_to_move = 1.0
# Distancia de inicio
current_distance = 0.0
# Posición de inicio
init_pos = None

def update_position(msg: Odometry):
    global current_distance, init_pos

    # Si aún no se ha establecido la posición inicial, se define
    if init_pos == None:
        init_pos = msg.pose.pose.position

    # Actualiza cuánto ha avanzado el robot respecto a la posición inicial
    delta_x = msg.pose.pose.position.x-init_pos.x
    delta_y = msg.pose.pose.position.y-init_pos.y

    current_distance = math.sqrt(delta_x**2 + delta_y**2)

if __name__ == '__main__':
    rospy.init_node('linea_recta_odom', anonymous = True)
    # Se subscribe a los temas de lectura y publicación
    sub = rospy.Subscriber('/odom', Odometry, update_position)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # Establece el comando de movimiento con la velocidad máxima
    twist = Twist()
    twist.linear.x = max_linear_speed
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo('Distancia actual recorrida: ' + str(current_distance))
        # Si aún no se ha completado el movimiento, se mantiene el comando
        if current_distance < distance_to_move:
            pub.publish(twist)
            r.sleep()
        else:
        # Si se ha completado, se detiene el robot y el programa
            twist.linear.x = 0.0
            pub.publish(twist)
            break