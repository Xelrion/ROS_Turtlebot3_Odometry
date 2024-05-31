#!/usr/bin/env python

import rospy
from mi_odometria_turtlebot.msg import TrianguloAction, TrianguloGoal, TrianguloFeedback
import actionlib

def feedback_callback(msg: TrianguloFeedback):
    current_movement = msg.current_movement
    movement_id = msg.movement_id
    movement_state = msg.movement_state
    feedback = 'Movimiento actual: {} {}. Progreso actual: {}'.format(current_movement, movement_id, movement_state)
    rospy.loginfo(feedback)

if __name__ == '__main__':
    rospy.init_node('triangulo_action_client', disable_signals=True)
    # Espera a que el servicio se inicie
    rospy.loginfo('Esperando al servidor triangulo_server...')
    triangulo = actionlib.SimpleActionClient('triangulo_server', TrianguloAction)
    triangulo.wait_for_server()
    rospy.loginfo('Servidor encontrado.')
    try:
        # Entrada de usuario
        side_length = float(input('Elige la longitud deseada para los lados del triángulo (metros):\n'))
        # Envía el mensaje de petición al servidor y espera a que termine la acción
        rospy.loginfo('Servidor conectado. Enviando petición de triángulo...')
        goal = TrianguloGoal(side_length=side_length)
        triangulo.send_goal(goal,feedback_cb=feedback_callback)
        triangulo.wait_for_result()
        success = triangulo.get_result()
        if success:
            rospy.loginfo('Respuesta recibida. El robot ha ejecutado el triángulo.')
        else:
            rospy.logwarn('Respuesta recibida. El robot no ha ejecutado el triángulo correctamente.')
    except KeyboardInterrupt as e:
        rospy.logwarn("El cliente se ha detenido. Enviado petición de parada al servidor...: %s"%e)
        triangulo.cancel_goal()