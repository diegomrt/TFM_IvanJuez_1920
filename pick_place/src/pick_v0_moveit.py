#!/usr/bin/env python
# coding=utf-8

"""
    Version 1 - 09-06-2020
    
    Command the irb120 arm and gripper to grasp a target object (box) and move it to a new
    location, using the pick() method and the GRASP message in MoveIt commander

    Included a second object (as obstacle) and a table

    Usage:
	1. Launch ABB IRB120 robot including MoveIt
	2. python "this_file.py"
    
    Some parts are taken from the "pick_and_place.py" Moveit demo of the Turtlebot arm:
    https://github.com/turtlebot/turtlebot_arm/tree/kinetic-devel/turtlebot_arm_moveit_demos/bin

    Diego Martín Martín - diego.martin.martin@urjc.es

"""

import sys
import rospy
import copy, math
import moveit_commander

from math import pi 
from moveit_commander import RobotCommander, MoveGroupCommander,PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import PlanningScene, ObjectColor, DisplayTrajectory, Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy

# Si activamos debug obtenemos info detallada de cada proceso
debug = True

# Constantes: Nombres de grupos y frames
GROUP_NAME_ARM = 'irb_120'
GROUP_NAME_GRIPPER = 'robotiq_85'
GRIPPER_FRAME = 'flange' 
FIXED_FRAME = 'world'

# Posiciones predefinidas para brazo. EL place se hace con IK, no con place() 
ARM_HOME_O = [0, pi/2, 0]		# HOME [roll, pitch, yaw] in FIXED_FRAME
ARM_HOME_P = [0.1, 0.2, 0.6]		# HOME [x,y,z] in FIXED_FRAME

ARM_PLACE_O = [0, 0, 0] 		# PLACE [roll, pitch, yaw] in FIXED_FRAME
ARM_PLACE_P = [0.4, -0.3, 0.6]		# PLACE [x,y,z] in FIXED_FRAME

# Posiciones predefinidas para pinza y otros parametros relacionados 
GRIPPER_OPEN = [0.0]
GRIPPER_cerrada = [0.6]
GRIPPER_JOINT_NAMES = ['gripper_finger1_joint']
GRASP_OVERTIGHTEN = 0.001	# Regula cuánto me paso apretando al coger (típico 2 mm) 
GRIPPER_EFFORT = [1.0]
GRIPPER_EXTRA = 0.15		# Distancia de la muñeca del robot al TCP  

# Parámetros para el mensaje de GRASP
VECTOR_PREGRASP = [0.0, 0.0, -1]	# Aprox PRE_GRASP en eje -z (de arriba hacia abajo)
VECTOR_POSTGRASP = [0.0, 0.0, 1]	# Alejamiento POST eje z (ahora hacia arriba)
APP_DISTANCE = 0.2
APP_MIN_DIST = 0.05
GRASP_FRAME = FIXED_FRAME

# Create a dictionary to hold object colors
colors = dict()

###################################################	
### FUNCIONES PARA CINEMÁTICA del brazo y la pinza  
###################################################

# IK ROBOT: Funcion para mover el TCP a una posicion y orientacion determinada (fuera de MoveIt)
def move_pose_arm(roll,pitch,yaw,x,y,z):
    pose_goal = Pose()
    quat = quaternion_from_euler(roll,pitch,yaw)
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    arm.set_pose_target(pose_goal)

    plan = arm.go(wait=True)

    arm.stop()
    arm.clear_pose_targets()

# Funcion sencilla para mover una herramienta de dos ejes de manera simétrica (fuera de MoveIt)
def move_joint_gripper(joint):
    joint_goal = gripper.get_current_joint_values()
    joint_goal[0] = joint
    #joint_goal[1] = joint

    gripper.go(joint_goal, wait=True)
    gripper.stop() # Garantiza que no hay movimiento residual


###################################################	
### FUNCIONES para trabajar con MOVEIT  
###################################################
    
# Función para crear las "pre_grasp_posture" y "grasp_posture" del mensaje "grasp"
# Contienen un conjunto de puntos para los ejes de la herramienta
# Se usa un mensaje tipo JointTrajectory, aunque típicamente no se necesitan muchos puntos
# Está función se llamará desde la función principal "make_grasps" que monta el msg "grasp"
# IMPORTANTE: Si la herramienta tiene dos ejes, hay que pasarle una lista de dos valores por punto.

def make_gripper_posture(joint_positions):
	t = JointTrajectory()
	t.joint_names = GRIPPER_JOINT_NAMES	
        tp = JointTrajectoryPoint()
        tp.positions = joint_positions
        tp.effort = GRIPPER_EFFORT	# Posiblemente no se está usando. Comprobar
	tp.time_from_start = rospy.Duration(1.0)
        t.points.append(tp)
        return t


# Función para grear el msg ROS de tipo "GripperTranslation" que se usa dentro del msg "grasp"
# Acepta el vector de aproximación, la distancia de aproximación y la mínima permitida
# para considerar que se puede hacer un grasp válido
# Esta función se llamará siempre desde la función "make_grasps"

def make_gripper_translation(vector, desired_dist, min_dist):
        g = GripperTranslation()
	g.direction.vector.x = vector[0]
	g.direction.vector.y = vector[1]
	g.direction.vector.z = vector[2]
        g.direction.header.frame_id = GRASP_FRAME # Importante: vector dado en GRASP_FRAME
        g.desired_distance = desired_dist
	g.min_distance = min_dist
        return g

# FUNCION PRINCIPAL PARA GENERAR UNO O VARIOS MENSAJES DE "GRASP" COMPLETO
# Acepta como parámetros:
#     1) POSICION INICIAL PARA EL GRASP
#     2) OBJETO A COGER (se eliminan las colisiones para él) 
#     3) APERTURA DE LA HERRAMIENTA AL COGER (variará de objeto a objeto)
# La generación de varios mensajes se controla dando valores a los angulos de:
#	PITCH y YAW
# Se hace depender la "calidad" del PITCH, se prefieren ángulos cercanos a cero
# Multiples grasps permiten coger cerca de obstáculos o de posiciones en el límite del espacio de trabajo

def make_grasps(initial_pose_stamped, allowed_touch_objects, grasp_opening=[0]):
	g = Grasp()
	g.grasp_pose = initial_pose_stamped
	g.allowed_touch_objects = allowed_touch_objects
	g.max_contact_force = 0			# Disabled si = 0

	# Abrimos pinza totalmente al inicio. Cerramos hasta valor dado al coger
	g.pre_grasp_posture = make_gripper_posture(GRIPPER_OPEN)
	#g.grasp_posture = make_gripper_posture(grasp_opening)
	g.grasp_posture = make_gripper_posture(grasp_opening)

	# Recordemos: vector de aproximacion, distancia deseada, distancia mínima
	g.pre_grasp_approach = make_gripper_translation(VECTOR_PREGRASP, APP_DISTANCE, APP_MIN_DIST)
	g.post_grasp_retreat = make_gripper_translation(VECTOR_POSTGRASP, APP_DISTANCE, APP_MIN_DIST)

	# ANGULO de roll FIJO 
	roll_fixed = 0
	
	# ANGULOS VARIABLES: SI SE INDICAN VARIOS CREA UN MENSAJE DE GRASP POR CADA UNO
	pitch_vals = [pi/2]
	yaw_vals = [0]

	# Generacion de un mensaje de grasp para cada angulo pitch y yaw
	grasps = []	
	for yaw in yaw_vals:
	    for pitch in pitch_vals:
		q = quaternion_from_euler(roll_fixed, pitch, yaw)
		g.grasp_pose.pose.orientation.x = q[0]
		g.grasp_pose.pose.orientation.y = q[1]
		g.grasp_pose.pose.orientation.z = q[2]
		g.grasp_pose.pose.orientation.w = q[3]
	
		g.id = str(len(grasps))
		g.grasp_quality = 1.0 - abs(pitch)		# Son preferibles pitchs próximos a cero!

		# Con .append se van creando componentes en la lista
		grasps.append(deepcopy(g))
	
	# Se devuelve la lista de uno o varios mensajes grasp creados
	return grasps

################################################################	
### FUNCIONES para trabajar cambiar color ESCENA (prescindibles)
################################################################
 
# Set the color of an object
def setColor(name, r, g, b, a=0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()

        # Set the id to the name given as an argument
        color.id = name

        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update the global color dictionary
        colors[name] = color

# Actually send the colors to MoveIt!
def sendColors():
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff
        p.is_diff = True

        # Append the colors from the global color dictionary
        for color in colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        scene_pub.publish(p)

if __name__=='__main__':

	###########################################################
	# BLOQUE 1: Inicialización: nodos, publish and suscribe...	
	###########################################################
	
	# Nodo, publisher and suscriber
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_place_node')
	scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
	gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)

      	# MoveIt stuff: brazo, pinza, robot, escena
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
	arm.allow_replanning(True)
        gripper = moveit_commander.MoveGroupCommander(GROUP_NAME_GRIPPER)    	
	
	# Para publicar trayectorias en RViz
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               DisplayTrajectory,
                                               queue_size=20) 
	
	# Frame de referencia y restricciones (tiempo, attempts)
	arm.set_pose_reference_frame(FIXED_FRAME)	
	arm.set_planning_time(10)		# 10 s para no tener TIME_OUTs en arm
	gripper.set_planning_time(5)		# 5 s para no tener TIME_OUTs en gripper
	max_pick_attempts = 1
	tol_gripper_o= gripper.get_goal_orientation_tolerance()
	tol_gripper_p= gripper.get_goal_position_tolerance()
	tol_arm_o= arm.get_goal_orientation_tolerance()
	tol_arm_p= arm.get_goal_position_tolerance()

	tol_gripper_o_n= gripper.set_goal_orientation_tolerance(0.01)
	tol_gripper_p_n= gripper.set_goal_position_tolerance(0.001)
	tol_arm_o_n= arm.set_goal_orientation_tolerance(0.01)
	tol_arm_p_n= arm.set_goal_position_tolerance(0.001)
	
	
	# Si queremos saber qué hay en la escena
	# objects_in_scene = scene.get_objects()
	
	# Limpiamos la escena (si existe de ejecuciones previas)
	box1_id='box1'	
	box2_id='box2'	
	table_id = 'table'
        scene.remove_world_object(box1_id)
	scene.remove_world_object(box2_id)
	scene.remove_world_object(table_id)
	rospy.sleep(1)

	# ROBOT A HOME y ABRO PINZA (esto no usa MoveIt)
        rospy.loginfo("Moving arm to HOME")	
	move_pose_arm(ARM_HOME_O[0],ARM_HOME_O[1],ARM_HOME_O[2],ARM_HOME_P[0],ARM_HOME_P[1],ARM_HOME_P[2])
        rospy.sleep(2)
        rospy.loginfo("Opening gripper")
        move_joint_gripper(GRIPPER_OPEN[0])

	if debug == True:
		# Detección del link que vamos a mover
	        eef = arm.get_end_effector_link()
		print "tolerancias iniciales del brazo y pinza= ", tol_arm_o, tol_arm_p, tol_gripper_o , tol_gripper_p
		print "tolerancias impuestas del brazo y pinza= ", tol_arm_o_n, tol_arm_p_n, tol_gripper_o_n , tol_gripper_p_n
	
		print "\n El End effector detectado es= ", eef
        	# Estado del robot
        	print "\n ============ Printing robot state ============"
        	print robot.get_current_state()
        	print "" 	
		raw_input("\n Robot listo y en HOME. BLOQUE 1 terminado. Press key to continue")

	################################################
	# BLOQUE 2: Creacion escena con una única caja
	################################################
       	
	# Dimensiones de las cajas y de la mesa [largo, ancho, altura]
	box1_size=[0.05, 0.05, 0.05]
	box2_size = [0.2, 0.05, 0.4]
        table_size = [0.3, 1, 0.02]

	# Fijo posicion y orientacion de la mesa. Se añade a la escena	
	table_pose = PoseStamped()
        table_pose.header.frame_id = robot.get_planning_frame()
        table_pose.pose.position.x = 0.3
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = 0.1
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

       	# Fijo posicion y orientacion de la caja 1 (z sobre la mesas siempre)
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = robot.get_planning_frame()
        box1_pose.pose.position.x = 0.3
        box1_pose.pose.position.y = 0.3
        box1_pose.pose.position.z = table_pose.pose.position.z + table_size[2]/2 + box1_size[2]/2
        scene.add_box(box1_id, box1_pose, box1_size)

	# Fijo posicion y orientacion de la caja 2 (obstáculo). Se añade a la escena
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = robot.get_planning_frame()
        box2_pose.pose.position.x = 0.3
        box2_pose.pose.position.y = 0
        box2_pose.pose.position.z = table_pose.pose.position.z + table_size[2]/2 + box2_size[2]/2
        scene.add_box(box2_id, box2_pose, box2_size)

	# Set the support surface name to the table object
        arm.set_support_surface_name(table_id)

	# Colores diferentes para objetos estáticos (opcional, pero queda bien)
        setColor(table_id, 0.8, 0, 0, 1.0) 
	setColor(box2_id, 0.9, 0.8, 0, 1.0)       
	sendColors()

	rospy.sleep(1)
        
	if debug == True:
		print "\n La pose de pick creada para ol objeto", box1_id,"es= ", box1_pose
		print "\n El planning frame es = ",  box1_pose.header.frame_id
		raw_input("\n BLOQUE 2 terminado. Press key to continue")

	###############################################
	# BLOQUE 3: Grasping de la caja 1 con pick()
	###############################################

	# Posición de la caja como objetivo del grasp. Sumamos extra por tamaño herramienta
	grasp_pose = box1_pose
	grasp_pose.pose.position.z = grasp_pose.pose.position.z + GRIPPER_EXTRA

	# Llamada generación mensajes de graps 
	#grasps = make_grasps(grasp_pose, [box1_id], [box1_size[1]/2 - GRASP_OVERTIGHTEN, box1_size[1]/2 - GRASP_OVERTIGHTEN])
	grasps = make_grasps(grasp_pose, [box1_id], [0.5 - box1_size[1]/2 - GRASP_OVERTIGHTEN] )

	# Publico las grasp poses para verlas en RVIZ
	for grasp in grasps:
	    gripper_pose_pub.publish(grasp.grasp_pose)
	    print "\n ------------------------------------------------------------------\n"
	    print "\n grasp =\n",grasp #debug 
	    rospy.sleep(0.2)
	# Pruebo exito/fallo y el numero de intentos para la operación de pick
	result = MoveItErrorCodes.FAILURE
	n_attempts = 0

	# PICK: Repito hasta que tenemos un buen resultado 
	while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
	    result = arm.pick(box1_id, grasps)
	    n_attempts += 1
	    rospy.loginfo("Pick intentos: " + str(n_attempts))
	    rospy.sleep(0.2)

	# Si el pick es exitoso, intento la locacilación del pick
	if result == MoveItErrorCodes.SUCCESS:
	    success = False
	    n_attempts = 0
	    
	else:
	    rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

	##################################################################
	# BLOQUE 4: Place. Por ahora sólo movimiento del brazo sin MoveIt
	##################################################################
	raw_input("\n BLOQUE 3 terminado. Press key to continue")
	# MUEVO ROBOT a punto de place CON OBJETO COGIDO (por ahora con IK clásica)
	# Uso la orientación del punto HOME (Place vertical)
	rospy.loginfo("Moving arm to PLACE point")	
	#move_pose_arm(ARM_HOME_O[0],ARM_HOME_O[1],ARM_HOME_O[2],box1_pose.pose.position.x, -0.3, box1_pose.pose.position.z)
	move_pose_arm(ARM_HOME_O[0],ARM_HOME_O[1],ARM_HOME_O[2],box1_pose.pose.position.x, -0.3, box1_pose.pose.position.z)
        rospy.sleep(0.5)
        
	# Abro pinza y suelto el objeto con remove_attached_object
	rospy.loginfo("Opening gripper")
        move_joint_gripper(GRIPPER_OPEN[0])
	scene.remove_attached_object(GRIPPER_FRAME, box1_id)
        rospy.sleep(0.5)	

	# ROBOT A HOME FINAL. Podría generar colisión al no estar bien hecho con place()
        rospy.loginfo("Moving arm to HOME")	
	move_pose_arm(ARM_HOME_O[0],ARM_HOME_O[1],ARM_HOME_O[2],ARM_HOME_P[0],ARM_HOME_P[1],ARM_HOME_P[2])
        	
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)

