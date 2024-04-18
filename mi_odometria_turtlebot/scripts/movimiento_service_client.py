#!/usr/bin/env python

import rospy
from mi_odometria_turtlebot.srv import MovLinRot, MovLinRotRequest

if __name__ == '__main__':
    rospy.init_node('movimiento_service_client')
    # Espera a que el servicio se inicie
    rospy.loginfo('Esperando al servidor movimiento_server...')
    rospy.wait_for_service('movimiento_server')
    rospy.loginfo('Servidor encontrado.')
    try:
        # Envía el mensaje de petición al servidor y espera a que termine la acción
        movimiento = rospy.ServiceProxy('movimiento_server', MovLinRot)
        rospy.loginfo('Servidor conectado. Enviando petición de línea recta, 1 metro...')
        success = movimiento(MovLinRotRequest(type=0,magnitude=1))
        if success:
            rospy.loginfo('Respuesta recibida. El robot se ha desplazado en línea recta.')
        else:
            rospy.logwarn('Respuesta recibida. El robot no ha avanzado correctamente.')
        rospy.loginfo('Enviando petición de rotación, 90º...')
        success = movimiento(MovLinRotRequest(type=1,magnitude=90))
        if success:
            rospy.loginfo('Respuesta recibida. El robot ha rotado 90º.')
        else:
            rospy.logwarn('Respuesta recibida. El robot no ha rotado correctamente.')
    except rospy.ServiceException as e:
        rospy.logwarn("La comunicación con el servidor ha fallado: %s"%e)
