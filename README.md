# TFM_IvanJuez_1920
TFM de Iván Juez - URJC

Se han realizado diferentes cambios en el repositorio para poder realizar la acción pick and place leyendo los datos obtenidos a través de la cámara y gazebo tanto en Rviz como en Gazebo.

Para lanzarlo en Rviz y Gazebo con el plugin utilizo el siguiente comando:

·roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit_rviz_v1.launch load_grasp_fix:=true

En el caso del estudio 1: se situa la cámara Kinect en posición cenital para el reconocimiento de objetos y se realiza un estudio de los resultados obtenidos para ello
se abre una nueva terminal donde se tratará la imagen obtenida con la cámara KInect a la cual se le realiza un filtro de color para la identificación de objetos y la posición de estos:

Si queremos localizar cajas o cilindros:

  ·rosrun estudio_1 posicion_caja_cil.py
  
Si queremos localizar esferas:

  ·rosrun estudio_1 posicion_esferas.py


Seguidamente, en una nueva terminal para realizar el pick se debe introducir los diferentes objetos que queramos incluir en la escena desde Gazebo (caja_grd, caja_peq, esfera_grd, esfera_peq) una vez creada la escena que se quiera, se lanza:

Si queremos coger cajas o cilindros:

  ·rosrun estudio_1 pick_caja_cil.py
  
Si queremos coger esferas:

  ·rosrun estudio_1 pick_esferas.py

Dentro de la terminal, se pide que objeto se quiere coger y directamente obtiene la posición y orientación en Gazebo y de la cámara Kinect para realizar el pick.






