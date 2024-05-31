#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class LineaRecta_Odom:
    # Variables de configuración internas
    # Número de mensajes de parada que se enviarán al robot para asegurar su detención
    _max_stop = 3
    # Posición de inicio del movimiento
    _initial_position = None
    # Distancia actual recorrida
    _current_distance = 0.0

    # Inicializa los parámetros, publishers y subscribers necesarios
    def __init__(self, max_linear_speed = 0.05, min_linear_speed = 0.01, p_regulator = 2, distance_to_move = 1.0):
        # Parámetros de configuración del movimiento
        self._max_linear_speed = max_linear_speed   # metros/segundo
        self._min_linear_speed = min_linear_speed   # metros/segundo
        self._p_regulator = p_regulator             # ganancia P para la regulación de velocidad
        self._distance_to_move = distance_to_move    # metros
        # Frecuencia de actualización del servidor
        self._r = rospy.Rate(10)    # Hz
        # Suscripción a la odometría
        self._sub = rospy.Subscriber('/odom', Odometry, self.update_position)
        # Publicación de comandos de movimiento
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
        # Publica el comando de velocidad hasta alcanzar la distancia deseada
        while self._current_distance < self._distance_to_move:
            # Envía comandos de avance al robot, adaptando la velocidad a la distancia restante
            twist.linear.x = max(min(self._p_regulator*(self._distance_to_move-self._current_distance), self._max_linear_speed), self._min_linear_speed)
            self._pub.publish(twist)
            # Información al usuario sobre el estado actual del movimiento
            rospy.loginfo('{current}m/{goal}m ({percent}%)'.format(
                goal = round(self._distance_to_move, 2),
                current = round(self._current_distance, 2),
                percent = round(100*self._current_distance/self._distance_to_move,2)
            ))
            self._r.sleep()
        # Tras completar el movimiento, detiene al robot
        self.movement_stop(self._max_stop)
        rospy.loginfo('''Movimiento completado con éxito.\n
                      El robot ha avanzado {distancia} metro/s.'''.format(
                          distancia = self._distance_to_move
                      ))

    # Actualiza la posición actual del robot y la distancia recorrida
    def update_position(self, msg:Odometry):
        # Si aún no se ha establecido la posición inicial, se define su valor
        if self._initial_position == None:
            self._initial_position = msg.pose.pose.position
        # Actualiza la distancia recorrida por el robot respecto a la posición inicial
        delta_x = msg.pose.pose.position.x-self._initial_position.x
        delta_y = msg.pose.pose.position.y-self._initial_position.y
        self._current_distance = math.sqrt(delta_x**2 + delta_y**2)

    # Envía varios mensajes de parada para asegurar que el robot los recibe y se detiene correctamente
    def movement_stop(self, num_msg = 3):
        # Genera el mensaje de parada y lo envía las veces solicitadas
        twist = Twist()
        rospy.logdebug('Deteniendo movimiento...')
        for i in range(num_msg):
            self._pub.publish(twist)
            self._r.sleep()
        rospy.logdebug('Movimiento detenido.')

if __name__== '__main__':
    try:
        rospy.init_node('move_robot_node', anonymous=True)
        rospy.loginfo('Nodo iniciado.\n')
        movimiento = LineaRecta_Odom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass