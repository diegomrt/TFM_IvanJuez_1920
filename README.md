# TFM_IvanJuez_1920
TFM de Iván Juez - URJC

He modificado los archivos denominandolos como _final para que los puedas ver y los identifiques correctamente. Esos ficheros modificados son:

·Dentro del directorio de ABB_experimental:

·abb_irb120_gazebo/urdf : aquí he modificado los ficheros irb120_3_58.xacro y irb120_3_58_macro.xacro. 

·abb_irb120_support/urdf : aquí he modificado los ficheros irb120_3_58.xacro y irb120_3_58_macro.xacro.

·Dentro del directorio de irb120_robotiq85:

·irb120_robotiq85_moveit_config/config : aquí he modificado el fichero irb120_robotiq85.srdf

·irb120_robotiq85_gazebo/urdf : aquí he modificado el fichero irb120_robotiq85_macro.xacro

·irb120_robotiq85_gazebo/launch : aquí he modificado el fichero irb120_robotiq85_gazebo_moveit_rviz.launch

De esta forma, ya he conseguido que me lanze correctamente el brazo y la pinza.

Para lanzarlo en Rviz y Gazebo utilizo el siguiente comando:

·roslaunch irb120_robotiq85_gazebo irb120_robotiq85_gazebo_moveit_rviz_final.launch


Para realizar el pick:

·rosrun pick_place pick_place.py

La otra alternativa para realizar el pick es lanzando:

·rosrun pick_place pick_place_prueba.py

