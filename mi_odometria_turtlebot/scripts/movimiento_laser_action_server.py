#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mi_odometria_turtlebot.msg import MovLinRotAction, MovLinRotGoal, MovLinRotResult, MovLinRotFeedback
from tf.transformations import euler_from_quaternion
import math
import actionlib

class MovimientoLaser_ActionServer:

    # Inicialización de variables internas
    _odom_position_x = 0.0
    _odom_position_y = 0.0
    _odom_orientation_yaw = 0.0
    # Número de veces que se envía el mensaje de parada al robot
    _max_stop = 3
    # Detección de obstáculos
    _path_free = True
    # Variable para el seguimiento de la ejecución correcta de la petición
    _success = True

    def __init__(self,
                 name='movimiento_action',
                 max_linear_speed=0.1,
                 min_linear_speed=0.01,
                 max_rotation_speed=0.5,
                 min_rotation_speed=0.1,
                 p_regulator = 1.5,
                 max_angle_diff=2,
                 max_security_distance = 0.25,
                 laser_angles_detection = 10):
        # Nombre del servidor de acción
        self._name = name
        # Parámetros de configuración del movimiento
        self._max_linear_speed = max_linear_speed
        self._min_linear_speed = min_linear_speed
        self._max_rotation_speed = max_rotation_speed
        self._min_rotation_speed = min_rotation_speed
        self._p_regulator = p_regulator
        self._max_angle_diff = math.radians(max_angle_diff)
        self._max_security_distance = max_security_distance
        self._laser_angles_detection = laser_angles_detection
        # Frecuencia de actualización del servidor
        self._r = rospy.Rate(10)
        # Suscripción a odometría, láser y publicación en movimiento
        self._sub = rospy.Subscriber('/odom', Odometry, self.update_position)
        self._subLaser = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # Inicia el servidor
        self._server = actionlib.SimpleActionServer('movimiento_server', MovLinRotAction, self.movement_callback, False)
        self._server.start()
        rospy.loginfo('Servidor de movimiento iniciado.')

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
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self._odom_orientation_yaw = yaw

    def laser_callback(self, msg:LaserScan):
        # Comprueba si hay un obstáculo frente al robot en el rango de ángulos solicitado frente a él
        #print(str(msg.ranges[0]) + ' ' + str(msg.ranges[89]) + ' ' + str(msg.ranges[179]) + ' ' + str(msg.ranges[269]))
        #print(str(msg.ranges[2]) + ' ' + str(msg.ranges[-2]))
        angle = 0
        laser_measurement = 0
        obstacle_detected = False
        while angle < self._laser_angles_detection:
            if msg.ranges[0-laser_measurement] < self._max_security_distance or msg.ranges[0+laser_measurement] < self._max_security_distance:
                obstacle_detected = True
                break
            angle += math.degrees(msg.angle_increment)
            laser_measurement += 1
        # Si se ha detectado obstáculo, se detiene el robot; si no, se reanuda la marcha
        if obstacle_detected and self._path_free == True:
            self._path_free = False
            rospy.loginfo('''Se ha detectado un obstáculo en el camino. El movimiento se detendrá hasta que sea seguro continuar.''')
        elif not obstacle_detected and self._path_free == False:
            self._path_free = True
            rospy.loginfo('''El obstáculo ya no está presente. Ahora, es seguro continuar.''')

    def movement_stop(self):
        # Genera el mensaje de parada y lo envía las veces solicitadas
        twist = Twist()
        rospy.logdebug('Deteniendo movimiento...')
        for i in range(self._max_stop):
            self._pub.publish(twist)
            self._r.sleep()
        rospy.logdebug('Movimiento detenido.')

    def movement_linear(self, distance_to_move):
        # Variables de seguimiento de posición y distancia recorrida
        initial_position_x = self._odom_position_x
        initial_position_y = self._odom_position_y
        current_distance = 0.0
        # Crea el mensaje de feedback
        feedback_msg = MovLinRotFeedback()
        # Establece el mensaje de movimiento con la velocidad lineal máxima
        rospy.loginfo('Iniciando movimiento: línea recta.')
        twist = Twist()
        twist.linear.x = self._max_linear_speed
        # Envía el mensaje hasta que el robot alcance su destino
        while current_distance < distance_to_move:
            # El robot solo se desplaza si es seguro continuar. En caso contrario, se detiene.
            if self._path_free:
                twist.linear.x = max(min(self._p_regulator*(distance_to_move-current_distance), self._max_linear_speed), self._min_linear_speed)
            else:
                twist.linear.x = 0
            self._pub.publish(twist)
            # Información al usuario sobre el estado actual del movimiento
            rospy.loginfo('{current}m/{goal}m ({percent}%)'.format(
                goal = round(distance_to_move, 2),
                current = round(current_distance, 2),
                percent = round(100*current_distance/distance_to_move,2)
            ))
            # Envía el feedback al cliente
            feedback_msg.current_movement = 'Línea recta'
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
        rospy.loginfo('Movimiento finalizado: línea recta.')

    def movement_rotational(self, rotation_angle):
        # Variables de seguimiento de orientación inicial y objetivo
        remaining_angle = math.radians(rotation_angle)
        initial_orientation_yaw = self._odom_orientation_yaw
        goal_orientation_yaw = initial_orientation_yaw + remaining_angle
        # Se ajusta el ángulo objetivo, ya que la odometría va de -180º a 180º
        if goal_orientation_yaw > math.radians(180):
            goal_orientation_yaw -= math.radians(360)
        # Crea el mensaje de feedback
        feedback_msg = MovLinRotFeedback()
        # Establece el mensaje de movimiento con la velocidad rotacional máxima
        twist = Twist()
        # Envía el mensaje hasta que el robot alcance la orientación objetivo
        while abs(remaining_angle) > self._max_angle_diff:
            twist.angular.z = math.copysign(1.0, rotation_angle) * max(min(self._p_regulator*(abs(remaining_angle)), self._max_rotation_speed), self._min_rotation_speed)
            self._pub.publish(twist)
            # Información al usuario sobre el estado actual del movimiento
            rospy.loginfo('{current}º/{goal}º --- Ángulo restante: ({remaining}º)'.format(
                goal = round(math.degrees(goal_orientation_yaw), 2),
                current = round(math.degrees(self._odom_orientation_yaw), 2),
                remaining = round(math.degrees(remaining_angle),2)
            ))
            # Envía el feedback al cliente
            feedback_msg.current_movement = 'Rotación'
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

    def movement_callback(self, msg: MovLinRotGoal):
        self._success = True
        # Movimiento lineal
        if msg.type == 0:
            rospy.loginfo('''Petición de movimiento recibida..\n
                      Tipo de movimiento: lineal.\n
                      Distancia solicitada: {distancia} m\n
                      Velocidad de movimiento: {velocidad} m/s'''.format(
                          distancia = msg.magnitude,
                          velocidad = self._max_linear_speed
                      ))
            self.movement_linear(msg.magnitude)
        # Movimiento rotacional
        elif msg.type == 1:
            rospy.loginfo('''Petición de movimiento recibida..\n
                      Tipo de movimiento: rotación.\n
                      Giro solicitado: {angulo} grados\n
                      Velocidad de movimiento: {velocidad} m/s'''.format(
                          angulo = msg.magnitude,
                          velocidad = self._max_rotation_speed
                      ))
            self.movement_rotational(msg.magnitude)
        # Envía una respuesta al cliente si la acción no se ha cancelado
        if self._success:
            rospy.loginfo('Petición de movimiento completada.')
            result = MovLinRotResult(success=True)
            self._server.set_succeeded(result)
    
if __name__ == '__main__':
    rospy.init_node('movimiento_action_server')
    server = MovimientoLaser_ActionServer()
    rospy.spin()