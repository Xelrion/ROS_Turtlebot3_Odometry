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
        # Entrada de usuario
        tipo_movimiento = int(input('''Elige el tipo de movimiento deseado:\n
                            0 = Lineal\n
                            1 = Rotación\n'''))
        magnitud_movimiento = float(input('''Elige la magnitud del movimiento deseado:\n
                            Lineal -> Metros\n
                            Rotación -> Grados sexagesimales\n'''))
        # Envía el mensaje de petición al servidor y espera a que termine la acción
        rospy.loginfo('Servidor conectado. Enviando petición de movimiento...')
        goal = MovLinRotGoal(type=tipo_movimiento, magnitude=magnitud_movimiento)
        movimiento.send_goal(goal,feedback_cb=feedback_callback)
        movimiento.wait_for_result()
        success = movimiento.get_result()
        if success:
            rospy.loginfo('Respuesta recibida. El robot se ha desplazado correctamente.')
        else:
            rospy.logwarn('Respuesta recibida. El robot no ha completado su movimiento.')
        rospy.loginfo('Enviando petición de rotación, 90º...')
    except KeyboardInterrupt as e:
        rospy.logwarn("El cliente se ha detenido. Enviado petición de parada al servidor...: %s"%e)
        movimiento.cancel_goal()
    except rospy.ServiceException as e:
        rospy.logwarn("La comunicación con el servidor ha fallado: %s"%e)