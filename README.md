# TFM_IvanJuez_1920
TFM de Iván Juez - URJC

Se han realizado diferentes cambios en el repositorio para poder realizar la acción pick and place tanto en Rviz como en Gazebo.

Para lanzarlo en Rviz y Gazebo utilizo el siguiente comando:

·roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit_rviz_v1.launch 

Si queremos cargar el brazo usando el grasp plugin de Gazebo, debemos introducir el siguiente comando:

·roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit_rviz_v1.launch load_grasp_fix:=true


Para realizar el pick:

·rosrun pick_place pick_v1_moveit_gazebo.py


Dentro de la carpeta Media se puede encontrar un video (pick_v1.mkv) donde se puede ver como se realiza el pick and place tanto en Rviz como Gazebo.
