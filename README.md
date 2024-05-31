# mi_odometria
## ROS2 - Humble

Estos paquetes se componen de 6 scripts y 4 tipos de mensaje personalizados. Los scripts ejecutan diversas secuencias de movimiento sobre un robot de tipo Turtlebot3, modelo Burguer.
### Diferencias respecto al paquete de ROS1 - Noetic
En este caso, se han creado dos paquetes diferentes:
- **mi_odometria**: contiene los scripts principales de ejecución, y compila mediante un script de configuración escrito también en python.
- **mi_odometria_interfaces**: contiene las definiciones de los mensajes que se utilizan en gran parte de los scripts. Como este compila mediante el archivo CMakeLists, como era habitual en ROS1, ha sido necesario crearlo aparte.
La funcionalidad de los scripts se ha intentando mantener de forma prácticamente idéntica, con algunos cambios menores en los mensajes de información al usuario.
El mayor cambio a nivel interno ha sido la introducción de ejecuciones multihilo, una funcionalidad más compleja pero personalizable implementada en ROS2. En este caso, para permitir la ejecución de varias funciones a la vez en un mismo nodo (por ejemplo, el bucle de movimiento del robot y las funciones de lectura de la odometría), se crea un grupo de callbacks reentrantes al que se añaden todas las funciones de publicación y suscripción del nodo, incluyendo acciones y servicios. Después, en lugar de utilizar el ejecutor por defecto para gestionar el nodo, utilizamos un ejecutor multihilo (para el que, en este script, hemos asignado dos hilos en paralelo). De esta manera, el ejecutor permite realizar varias tareas en paralelo, cada una en su propio hilo.

### Línea recta, seguimiento por tiempo
- *lineaRecta_tiempo*
El turtlebot se desplaza en línea recta, recorriendo 1 metro de distancia. El seguimiento se realiza mediante tiempo: en cada ciclo de actualización, se multiplica la velocidad solicitada al robot por el tiempo transcurrido, estimando la distancia recorrida en ese intervalo.
### Línea recta, seguimiento por odometría
- *lineaRecta_odom*
El turtlebot se desplaza en línea recta, recorriendo 1 metro de distancia. El seguimiento se realiza mediante odometría, utilizando los sensores propioceptivos del robot, mejorando considerablemente la precisión respecto al tiempo. En cada ciclo, se calcula la diferencia entre la posición actual y la inicial; cuando esta distancia sea igual o superior a 1 metro, el robot se detendrá.
### Línea recta, servicio, seguimiento por odometría
- *lineaRecta_serviceServer*
- *lineaRecta_serviceClient*
- *Empty.srv*
Mismo funcionamiento que el script anterior. En este caso, se inicia un servidor de servicio que se encarga de realizar el movimiento. La ejecución del cliente permite llamar al servicio para que lleve a cabo la instrucción. Se podrá llamar al servicio cuantas veces sea necesario, sin necesidad de reiniciarlo, hasta que su respectivo nodo sea cerrado.
### Movimiento, servicio
- *movimiento_serviceServer*
- *movimiento_serviceClient*
- *MovLinRot.srv*
Este servicio implementa la posibilidad de escoger entre dos tipos de movimiento: lineal o rotación, y la magnitud del movimiento deseado para cada uno de ellos (metros en lineal, grados sexagesimales en rotación). La rotación se realiza mediante odometría: la orientación final se obtiene sumando la inicial del robot y el giro deseado. En cada iteración, esta se compara con la orientación actual. Si la diferencia entre ambas es inferior a un margen de error definido, el movimiento finalizará. También permite rotar en sentido horario y antihorario, según el signo de la rotación pedida.
### Movimiento, acción
- *movimiento_actionServer*
- *movimiento_actionClient*
- *MovLinRot.action*
Mismo funcionamiento que el script anterior. En este caso, se emplea una acción en lugar de un servicio, de manera que el cliente va recibiendo continuamente información sobre el estado actual del desplazamiento.
### Triángulo, acción
- *triangulo_actionServer*
- *triangulo_actionClient*
- *Triangulo.action*
Esta acción realiza un desplazamiento en forma de triángulo. Para ello, intercala tres avances lineales y tres rotaciones de 120º. Desde el cliente, se puede especificar la longitud deseada para los lados del triángulo.
### Polígono, acción
- *poligono_actionServer*
- *poligono_actionClient*
- *Poligono.action*
Esta acción realiza un desplazamiento en forma de polígono. Actúa de manera similar al triángulo, intercalando movimientos lineales y rotaciones. La longitud y el número de lados se definen desde la petición del cliente. Al inicio del movimiento, se calcula el ángulo de rotación necesario entre cada lado del polígono: 360º divididos entre el número de lados. Para el caso del triángulo, por ejemplo: 360º/3 = 120º. En el feedback, además, también se realiza un seguimiento del lado o vértice que el robot está realizando en cada momento.
### Movimiento, acción, detección de obstáculos
- *movimientoLaser_actionServer*
- *movimientoLaser_actionClient*
- *MovLinRot.action*
Mismo funcionamiento que el script de acción de movimiento. En este caso, se implementa una funcionalidad adicional: durante los desplazamientos lineales, el robot escanea la zona frontal mediante su láser. Si detecta algún obstáculo a una distancia inferior de un determinado margen de seguridad, se detendrá hasta que el área vuelva a estar libre.

### Parámetros de configuración generales
Los scripts implementan una serie de parámetros de configuración, que permiten alterar el comportamiento de los nodos a conveniencia. Estos parámetros se pueden alterar al crear una instancia de la clase correspondiente. Cuanto más complejos son los scripts, más opciones de configuración implementan.
A continuación, se muestran estos parámetros y su utilidad en los scripts presentados:
- **velocidad_lineal_maxima, velocidad_lineal_minima**: velocidad máxima y mínima a la que el robot se desplazará durante un movimiento de avance lineal, medida en m/s.
- **velocidad_rotacion_maxima, velocidad_rotacion_minima**: velocidad máxima y mínima a la que el robot se desplazará durante las rotaciones.
- **regulador_p**: ganancia del regulador P. Durante su movimiento, la velocidad del robot se calculará como este valor P multiplicado por la distancia restante (metros en lineal, radianes en rotación). De esta manera, cuando el movimiento restante sea grande, el robot avanzará a gran velocidad; cuando se vaya acercando a la posición objetivo, se frenará progresivamente para evitar frenazos bruscos y mejorar su precisión final. Esta velocidad nunca será mayor que la máxima definida, ni disminuirá por debajo de la inferior.
- **max_dif_angulo**: tolerancia de error en los movimientos de rotación, medida en grados sexagesimales. Cuando el robot rote, si la diferencia entre la orientación actual y la objetivo es inferior a este valor, se detendrá y terminará el giro. Reducir este valor mejora la precisión del giro, pero se corre el riesgo de que el robot no actualice su posición a tiempo y pase de largo. Esto se puede evitar reduciendo la velocidad de rotación o aumentando la frecuencia de actualización de posición (aunque nunca podrá ser menor a la de la odometría). Un valor mayor de tolerancia aumentará el error de giro, pero evitará que el robot se pase de largo a altas velocidades o bajas frecuencias de actualización.
- **tiempo_periodo**: periodo de actualización del robot, medido en segundos. Cuando el robot se encuentre en un bucle de desplazamiento, este será el periodo de tiempo que transcurrirá entre cada comprobación de posición. Un menor periodo permite mejorar la precisión en los desplazamientos, pero aumenta el consumo de CPU.
- **max_security_distance**: distancia máxima de seguridad respecto a obstáculos externos, medida en metros. Si el láser del robot detecta un obstáculo frente a él a una distancia menor, detendrá su movimiento hasta que este desaparezca. Es importante tener en cuenta la distancia de frenado del robot.
- **laser_angles_detection**: cono de detección frente al robot, en grados sexagesimales, en el que se buscarán obstáculos cercanos. Un valor de 5 grados, por ejemplo, buscará obstáculos en un cono de 10 grados de amplitud: 5 a la izquierda, y 5 a la derecha. Un cono 0 solo buscará en la medición del láser que hay justo frente al robot. El turtlebot3 burguer dispone de un láser de 360 medidas, una por cada grado sexagesimal.
