#!/usr/bin/env python

import rospy
from mi_odometria_turtlebot.msg import PoligonoAction, PoligonoGoal, PoligonoFeedback
import actionlib

def feedback_callback(msg: PoligonoFeedback):
    current_movement = msg.current_movement
    movement_id = msg.movement_id
    movement_state = msg.movement_state
    feedback = 'Movimiento actual: {} {}. Progreso actual: {}'.format(current_movement, movement_id, movement_state)
    rospy.loginfo(feedback)

if __name__ == '__main__':
    rospy.init_node('poligono_action_client', disable_signals=True)
    # Espera a que el servicio se inicie
    rospy.loginfo('Esperando al servidor poligono_server...')
    poligono = actionlib.SimpleActionClient('poligono_server', PoligonoAction)
    poligono.wait_for_server()
    rospy.loginfo('Servidor encontrado.')
    try:
        # Envía el mensaje de petición al servidor y espera a que termine la acción
        rospy.loginfo('Servidor conectado.')
        side_length = float(input('Ingrese longitud de lado del polígono:\n'))
        side_amount = int(input('Ingrese número de lados del polígono:\n'))
        rospy.loginfo('Enviando petición al servidor: polígono de {} lados, con longitud de lado: {}'.format(side_amount, side_length))
        goal = PoligonoGoal(side_length=side_length, side_amount=side_amount)
        poligono.send_goal(goal,feedback_cb=feedback_callback)
        poligono.wait_for_result()
        success = poligono.get_result()
        if success:
            rospy.loginfo('Respuesta recibida. El robot ha ejecutado el polígono.')
        else:
            rospy.logwarn('Respuesta recibida. El robot no ha ejecutado el polígono correctamente.')
    except KeyboardInterrupt as e:
        rospy.logwarn("El cliente se ha detenido. Enviado petición de parada al servidor...: %s"%e)
        poligono.cancel_goal()