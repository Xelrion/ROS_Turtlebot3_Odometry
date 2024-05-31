#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class LineaRecta_Tiempo:

    # Variables de configuración internas
    # Número de mensajes de parada que se enviarán al robot para asegurar su detención
    _max_stop = 3

    # Inicializa los parámetros, publishers y subscribers necesarios
    def __init__(self, max_linear_speed = 0.05, distance_to_move = 1.0):
        # Parámetros de configuración del movimiento
        self._max_linear_speed = max_linear_speed   # metros/segundo
        self._distance_to_move = distance_to_move    # metros
        # Frecuencia de actualización del servidor
        self._r = rospy.Rate(10)    # Hz
        # Publicación en movimiento
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Comunicación con el usuario
        rospy.loginfo('''Movimiento lineal iniciado.\n
                      Modo de control: odometría.\n
                      Distancia solicitada: {distancia} m\n
                      Velocidad de movimiento: {velocidad} m/s'''.format(
                          distancia = self._distance_to_move,
                          velocidad = self._max_linear_speed
                      ))
        # Inicia el movimiento
        self.move_robot()

    # Controla la ejecución del bucle principal de movimiento
    def move_robot(self):
        # Creación del mensaje de comando de velocidad
        twist = Twist()
        twist.linear.x = self._max_linear_speed
        # Inicialización del contador de tiempo
        start_time = rospy.get_time()
        goal_time = start_time + (self._distance_to_move/self._max_linear_speed)
        current_time = start_time
        # Publica el comando de velocidad hasta alcanzar la distancia deseada
        while current_time < goal_time:
            current_time = rospy.get_time()
            # Envía comandos de avance al robot
            self._pub.publish(twist)
            # Información al usuario sobre el estado actual del movimiento
            rospy.loginfo('{current}s/{goal}s ({percent}%)'.format(
                goal = round(goal_time - start_time, 2),
                current = round(current_time - start_time, 2),
                percent = round(100*(current_time - start_time)/(goal_time - start_time),2)
            ))
            self._r.sleep()
        # Tras completar el movimiento, detiene al robot
        self.movement_stop(self._max_stop)
        rospy.loginfo('''Movimiento completado con éxito.\n
                      El robot ha avanzado {distancia} metro/s.'''.format(
                          distancia = self._distance_to_move
                      ))

    # Envía varios mensajes de parada para asegurar que el robot los recibe y se detiene correctamente
    def movement_stop(self, num_msg = 3):
        # Genera el mensaje de parada y lo envía las veces solicitadas
        twist = Twist()
        rospy.logdebug('Deteniendo movimiento...')
        for i in range(num_msg):
            self._pub.publish(twist)
            self._r.sleep()
        rospy.logdebug('Movimiento detenido.')

if __name__ == '__main__':
    try:
        rospy.init_node('move_robot_node', anonymous=True)
        rospy.loginfo('Nodo iniciado.\n')
        movimiento = LineaRecta_Tiempo()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass