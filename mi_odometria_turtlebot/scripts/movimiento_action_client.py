#!/usr/bin/env python

import rospy
from mi_odometria_turtlebot.msg import MovLinRotAction, MovLinRotGoal, MovLinRotFeedback
import actionlib

def feedback_callback(msg: MovLinRotFeedback):
    current_movement = msg.current_movement
    movement_state = msg.movement_state
    feedback = 'Movimiento actual: {}. Progreso actual: {}'.format(current_movement, movement_state)
    rospy.loginfo(feedback)

if __name__ == '__main__':
    rospy.init_node('movimiento_action_client', disable_signals=True)
    # Espera a que el servicio se inicie
    rospy.loginfo('Esperando al servidor movimiento_server...')
    movimiento = actionlib.SimpleActionClient('movimiento_server', MovLinRotAction)
    movimiento.wait_for_server()
    rospy.loginfo('Servidor encontrado.')
    try:
        # Envía el mensaje de petición al servidor y espera a que termine la acción
        rospy.loginfo('Servidor conectado. Enviando petición de línea recta, 1 metro...')
        goal = MovLinRotGoal(type=0, magnitude=1)
        movimiento.send_goal(goal,feedback_cb=feedback_callback)
        movimiento.wait_for_result()
        success = movimiento.get_result()
        if success:
            rospy.loginfo('Respuesta recibida. El robot se ha desplazado en línea recta.')
        else:
            rospy.logwarn('Respuesta recibida. El robot no ha avanzado correctamente.')
        rospy.loginfo('Enviando petición de rotación, 90º...')
        goal = MovLinRotGoal(type=1, magnitude=90)
        movimiento.send_goal(goal,feedback_cb=feedback_callback)
        movimiento.wait_for_result()
        success = movimiento.get_result()
        if success:
            rospy.loginfo('Respuesta recibida. El robot ha rotado 90º.')
        else:
            rospy.logwarn('Respuesta recibida. El robot no ha rotado correctamente.')
    except KeyboardInterrupt as e:
        rospy.logwarn("El cliente se ha detenido. Enviado petición de parada al servidor...: %s"%e)
        movimiento.cancel_goal()