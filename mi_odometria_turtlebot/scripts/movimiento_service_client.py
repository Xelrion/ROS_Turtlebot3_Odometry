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
        # Entrada de usuario
        tipo_movimiento = int(input('''Elige el tipo de movimiento deseado:\n
                            0 = Lineal\n
                            1 = Rotación\n'''))
        magnitud_movimiento = float(input('''Elige la magnitud del movimiento deseado:\n
                            Lineal -> Metros\n
                            Rotación -> Grados sexagesimales\n'''))
        # Envía el mensaje de petición al servidor y espera a que termine la acción
        movimiento = rospy.ServiceProxy('movimiento_server', MovLinRot)
        rospy.loginfo('Servidor conectado. Enviando petición de movimiento...')
        success = movimiento(MovLinRotRequest(type=tipo_movimiento,magnitude=magnitud_movimiento))
        if success:
            rospy.loginfo('Respuesta recibida. El robot se ha desplazado correctamente.')
        else:
            rospy.logwarn('Respuesta recibida. El robot no ha avanzado correctamente.')
    except rospy.ServiceException as e:
        rospy.logwarn("La comunicación con el servidor ha fallado: %s"%e)
