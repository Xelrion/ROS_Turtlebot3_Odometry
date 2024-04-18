#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    rospy.init_node('move_robot_node', anonymous=True)
    rate = rospy.Rate(10)   # Frecuencia de publicación de comandos de velocidad

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist = Twist()

    # Velocidad lineal máxima (en m/s)
    max_linear_speed = 0.05
    # Distancia a avanzar (en metros)
    distance_to_move = 1.0

    # Iniciar movimiento hacia adelante
    twist.linear.x = max_linear_speed

    # Tiempo necesario para avanzar la distancia deseada
    time_to_move = distance_to_move/max_linear_speed
    rospy.loginfo('tiempo a mover: ' + str(time_to_move))

    # Publica el comando de velocidad hasta alcanzar la distancia deseada
    start_time = rospy.get_time()
    rospy.loginfo('tiempo a mover: ' + str(start_time))
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        rospy.loginfo( 'tiempo_actual: ' + str(current_time))
        elapsed_time = current_time - start_time

        if elapsed_time >= time_to_move:
            # Detiene el movimiento
            twist.linear.x = 0
            pub.publish(twist)
            rospy.loginfo("Robot ha avanzado 1 metro.")
            break

        pub.publish(twist)
        rospy.loginfo( 'avance: ' + str(twist.linear.x))
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass