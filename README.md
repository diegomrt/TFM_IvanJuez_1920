# TFM_IvanJuez_1920
TFM de Iván Juez - URJC

Se han realizado diferentes cambios en el repositorio para poder realizar la acción pick and place leyendo los datos obtenidos a través de la cámara y gazebo tanto en Rviz como en Gazebo.

·Simulación en 2D(OPENCV):

Para lanzarlo en Rviz y Gazebo con el plugin que permite realizar el pick se utiliza el siguiente comando:

     ·roslaunch irb120_robotiq85_gazebo TFM_ivan_2D.launch load_grasp_fix:=true

Al lanzar dicho comando, se observa la escena vacia en GAZEBO, se aprecia que la cámara se encuentra en posición cenital. Para poder llevar acabo la correcta simulación, en este caso se debe introducir las cajas de los tamaños y colores que queramos y para poder realizar el filtro de color debemos introducir lo siguiente en otra terminal:

     ·rosrun estudio_1 posicion_2D.py

Con este comando, nos aparecerá en la pantalla un input donde se debe escribir el color de la pieza que se desee: verde,azul o rojo. De esta forma comienza a funcionar el filtro y muestra en pantalla la pieza más grande y su contorno.

Seguidamente, para poder realizar el pick se debe introducir en una nueva terminal los diferentes objetos que queramos incluir en la escena desde Gazebo (caja_grd_color, caja_peq_color,) una vez creada la escena que se quiera, se lanza:

Si queremos coger cajas o cilindros:

     ·rosrun estudio_1 pick_2D.py
  
Ahora nos aparecera en terminal el input donde debemos escribir el nombre de la pieza deseada y autaticamente nos detecta la posición de esta a través de la KInect y la orientación de la pieza la obtenemos a través de Gazebo.


·Simulación en 3D(PCL): 

Para lanzarlo en Rviz y Gazebo con el plugin que permite realizar el pick se utiliza el siguiente comando:

     ·roslaunch irb120_robotiq85_gazebo TFM_ivan_3D.launch load_grasp_fix:=true

Al lanzar dicho comando, se vuelve a observar la escena vacia en GAZEBO, se aprecia que la cámara se encuentra en una posición diferente a la anterior simulación, en este caso tiene un ángulo de 45º respecto a la horizontal con el fin de captar una mayor superficie de las piezas. Para poder llevar acabo la correcta simulación, en este caso se debe introducir las piezas de los tamaños y colores que queramos y para este caso nos debemos ir a RVIZ e incluir que TF nos de los nombres de los frames, también podemos añadir la variable de POINTCLOUD2 con el topic de "/camera/depth/point" para ver la escena correctamente, en este caso nos debería señalar el correcto centro de masas de las piezas y reconocerlas perfectamente, pero aquí es donde falla el código.

Si escribimos en otra terminal rosnode list podemos ver como si se está lanzando el nodo de 'object detection' por lo que si está funcionando el código.

No obstante, si quieres ver el código se encuentra en /catkin_ws/src/TFM_IvanJuez_1920/pick_place/src --> example.cpp

UNa vez que me detecte bien las piezas, según entiendo en el código lo almacena dentro del topic /TF, como green_sphere,etc... y una vez me lo detecte correctamente y pueda leer la posición me quedaría realizar el pick correctamente en 3D, pero creo que eso una vez detectada la posición no creo que me cueste mucho.









