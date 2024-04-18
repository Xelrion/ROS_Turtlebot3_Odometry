#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mi_odometria_turtlebot.msg import TrianguloAction, TrianguloGoal, TrianguloResult, TrianguloFeedback
from tf.transformations import euler_from_quaternion
import math
import actionlib

class Triangulo_ActionServer:

    # Inicialización de variables internas
    _odom_position_x = 0.0
    _odom_position_y = 0.0
    _odom_orientation_yaw = 0.0
    # Número de veces que se envía el mensaje de parada al robot
    _max_stop = 3
    # Secuencia de movimientos a realizar para ejecutar el triángulo: 0, lineal; 1, rotación
    _movement_sequence = [0,1,0,1,0,1]
    # Variables para el seguimiento de la ejecución correcta de la petición
    _linear_movement_id = 0
    _rotation_movement_id = 0
    _success = True

    def __init__(self, name='triangulo_action', max_linear_speed=0.05, max_rotation_speed=0.5, max_angle_diff=5):
        # Nombre del servidor de servicio
        self._name = name
        # Parámetros de configuración del movimiento
        self._max_linear_speed = max_linear_speed
        self._max_rotation_speed = max_rotation_speed
        self._max_angle_diff = math.radians(max_angle_diff)
        # Frecuencia de actualización del servidor
        self._r = rospy.Rate(10)
        # Suscripción a odometría y publicación en movimiento
        self._sub = rospy.Subscriber('/odom', Odometry, self.update_position)
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Inicia el servidor
        self._server = actionlib.SimpleActionServer('triangulo_server', TrianguloAction, self.movement_callback, False)
        self._server.start()
        rospy.loginfo('Servidor de triángulo iniciado.')

    def is_preempt_requested(self):
        # Comprueba si el cliente ha solicitado parar la acción
        if self._server.is_preempt_requested():
            rospy.logwarn('El cliente ha solicitado detener la acción.')
            self.movement_stop()
            self._success = False
            self._server.set_preempted()
            return True
        return False

    def update_position(self, msg: Odometry):
        # Actualiza la posición del robot sobre el plano xy
        self._odom_position_x = msg.pose.pose.position.x
        self._odom_position_y = msg.pose.pose.position.y
        
        # Obtiene la orientación del robot sobre el eje z a partir de cuaternios
        orientation_q = msg.pose.pose.orientation
        orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self._odom_orientation_yaw = yaw

    def movement_stop(self):
        # Genera el mensaje de parada y lo envía las veces solicitadas
        twist = Twist()
        rospy.logdebug('Deteniendo movimiento...')
        for i in range(self._max_stop):
            self._pub.publish(twist)
            self._r.sleep()
        rospy.logdebug('Movimiento detenido.')

    def movement_linear(self, distance_to_move):
        # Actualiza el id del movimiento actual
        self._linear_movement_id += 1
        # Variables de seguimiento de posición y distancia recorrida
        initial_position_x = self._odom_position_x
        initial_position_y = self._odom_position_y
        current_distance = 0.0
        # Crea el mensaje de feedback
        feedback_msg = TrianguloFeedback()
        # Establece el mensaje de movimiento con la velocidad lineal máxima
        rospy.loginfo('Iniciando movimiento: lado.')
        twist = Twist()
        twist.linear.x = self._max_linear_speed
        # Envía el mensaje hasta que el robot alcance su destino
        while current_distance < distance_to_move:
            rospy.logdebug('Distancia actual recorrida: ' + str(current_distance))
            self._pub.publish(twist)
            # Envía el feedback al cliente
            feedback_msg.current_movement = 'Lado'
            feedback_msg.movement_id = self._linear_movement_id
            feedback_msg.movement_state = current_distance
            self._server.publish_feedback(feedback_msg)
            self._r.sleep()
            # Comprueba si el cliente ha solicitado detener la acción
            if self.is_preempt_requested():
                return
            # Actualiza la distancia recorrida
            delta_x = self._odom_position_x-initial_position_x
            delta_y = self._odom_position_y-initial_position_y
            current_distance = math.sqrt(delta_x**2 + delta_y**2)
        # Detiene el robot al terminar
        self.movement_stop()
        rospy.loginfo('Movimiento finalizado: lado.')

    def movement_rotational(self, rotation_angle):
        # Actualiza el id del movimiento actual
        self._rotation_movement_id += 1
        # Variables de seguimiento de orientación inicial y objetivo
        remaining_angle = math.radians(rotation_angle)
        initial_orientation_yaw = self._odom_orientation_yaw
        goal_orientation_yaw = initial_orientation_yaw + remaining_angle
        # Se ajusta el ángulo objetivo, ya que la odometría va de -180º a 180º
        if goal_orientation_yaw > math.radians(180):
            goal_orientation_yaw -= math.radians(360)
        # Crea el mensaje de feedback
        feedback_msg = TrianguloFeedback()
        # Establece el mensaje de movimiento con la velocidad rotacional máxima
        rospy.loginfo('Iniciando movimiento: rotación.')
        twist = Twist()
        twist.angular.z = self._max_rotation_speed
        # Envía el mensaje hasta que el robot alcance la orientación objetivo
        while abs(remaining_angle) > self._max_angle_diff:
            rospy.logdebug('Ángulo de rotación restante: ' + str(math.degrees(remaining_angle)))
            self._pub.publish(twist)
            # Envía el feedback al cliente
            feedback_msg.current_movement = 'Rotación'
            feedback_msg.movement_id = self._rotation_movement_id
            feedback_msg.movement_state = math.degrees(remaining_angle)
            self._server.publish_feedback(feedback_msg)
            self._r.sleep()
            # Comprueba si el cliente ha solicitado detener la acción
            if self.is_preempt_requested():
                return
            # Actualiza el ángulo de rotación restante
            remaining_angle = goal_orientation_yaw - self._odom_orientation_yaw
        # Detiene el robot al terminar
        self.movement_stop()
        rospy.loginfo('Movimiento finalizado: rotación.')

    def movement_callback(self, msg: TrianguloGoal):
        rospy.loginfo('Petición de movimiento recibida.')
        self._linear_movement_id = 0
        self._rotation_movement_id = 0
        self._success = True
        # Lee la longitud del lado del triángulo
        side_length = msg.side_length
        # Ejecuta la secuencia de movimientos del triángulo
        for mov in self._movement_sequence:
            if mov == 0:
                self.movement_linear(side_length)
            elif mov == 1:
                self.movement_rotational(120)
            # Si el cliente ha solicitado cancelar la acción, rompe el bucle
            if self._server.is_preempt_requested():
                break
        # Envía una respuesta al cliente si la acción no se ha cancelado
        if self._success:
            rospy.loginfo('Petición de triángulo completada.')
            result = TrianguloResult(success=True)
            self._server.set_succeeded(result)
    
if __name__ == '__main__':
    rospy.init_node('triangulo_action_server')
    server = Triangulo_ActionServer()
    rospy.spin()