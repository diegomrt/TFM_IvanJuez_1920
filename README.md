# TFM_IvanJuez_1920
TFM de Iván Juez - URJC

Se han realizado diferentes cambios en el repositorio para poder realizar la acción pick and place tanto en Rviz como en Gazebo.

Para lanzarlo en Rviz y Gazebo utilizo el siguiente comando:

·roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit_rviz_v1.launch 

Si queremos cargar el brazo usando el grasp plugin de Gazebo, debemos introducir el siguiente comando:

·roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit_rviz_v1.launch load_grasp_fix:=true


Para realizar el pick se debe introducir que caja quieres que coga o "verde" o "azul" y directamente obtiene la posición y orientación de las cajas en Gazebo para realizar el pick. Me falta por hacer la lista de objetos para que me diga autimaticamente los objetos que hay pero de momento funciona unicamente indicando la caja que quieres que coja.

·rosrun pick_place pick_v3_moveit_gazebo.py


