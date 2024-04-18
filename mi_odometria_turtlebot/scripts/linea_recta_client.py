#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest

if __name__ == '__main__':
    rospy.init_node('linea_recta_server')
    # Espera a que el servicio se inicie
    rospy.loginfo('Esperando al servidor linea_recta...')
    rospy.wait_for_service('linea_recta')
    rospy.loginfo('Servidor encontrado.')
    try:
        # Envía el mensaje de petición al servidor y espera a que termine la acción
        linea_recta = rospy.ServiceProxy('linea_recta', Empty)
        rospy.loginfo('Servidor conectado. Enviando petición...')
        response = linea_recta(EmptyRequest())
        rospy.loginfo("Respuesta recibida. El robot se ha desplazado en línea recta.")
    except rospy.ServiceException as e:
        rospy.logwarn("La comunicación con el servidor ha fallado: %s"%e)