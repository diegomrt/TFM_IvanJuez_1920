#!/usr/bin/env python
# coding=utf-8

#Defino todo lo que necesito de estas librerias
import sys
import rospy
import copy, math
from math import pi 
import moveit_commander
import actionlib
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import moveit_msgs.msg
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, MoveItErrorCodes, PickupAction, PickupGoal, PickupResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
from copy import deepcopy

#Defino el nombre del robot
ROBOT_NAME = "irb120_robotiq85"


if ROBOT_NAME == "irb120_robotiq85":
#Defino los diferentes grupos de MOVEGROUP como son el brazo por un lado y la pinza
    GROUP_NAME_ARM = 'irb_120'
    GROUP_NAME_GRIPPER = 'robotiq_85'

    GRIPPER_FRAME = 'gripper_base_link' #mi duda es si aqui debo poner donde se fija el dedo "gripper_finger1_joint" o directamente sería el mismo

    FIXED_FRAME = 'base_link'
#Defino las posiciones de la pinza en función de la posición del joint finger 1
    GRIPPER_CLOSED = [0.3]
    GRIPPER_OPEN = [0.0]
    GRIPPER_NEUTRAL = [0.03]
#En este caso defino el joint que se va a mover de la pinza 
    GRIPPER_JOINT_NAMES = ['gripper_finger1_joint']

    #GRASP_OVERTIGHTEN = 0.002
    GRASP_OVERTIGHTEN = 0.002
    GRIPPER_EFFORT = [1.0]

    max_pick_attempts = 5
###################################################	
### FUNCIONES QUE IRE LLAMANDO CUANDO NECESITE  
###################################################


# IK ROBOT: Funcion para mover el TCP a una posicion y orientacion determinada
def move_pose_arm(roll,pitch,yaw,x,y,z):
    pose_goal = geometry_msgs.msg.Pose()
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

# Funcion para mover la herramienta 
def move_joint_gripper(joint):
    joint_goal = gripper.get_current_joint_values()
    joint_goal[0] = joint
    joint_goal[1] = joint

    gripper.go(joint_goal, wait=True)
    gripper.stop() # Garantiza que no hay movimiento residual

def createGrasp(grasp_pose):
    grasp = Grasp()
    grasp.id = "prueba"   
    grasp.grasp_pose = grasp_pose
    
    grasp.pre_grasp_posture.header.frame_id = "base_link" 
    grasp.pre_grasp_posture.header.stamp = rospy.Time.now() 
    grasp.pre_grasp_posture.joint_names = ["gripper_finger1_joint", "gripper_finger2_joint"]
    pos = JointTrajectoryPoint() 
    pos.positions.append(0.0)
    pos.positions.append(0.0)
    
    grasp.pre_grasp_posture.points.append(pos)
     
     
    grasp.grasp_posture.header.frame_id = "base_link"
    grasp.grasp_posture.header.stamp = rospy.Time.now() 
    grasp.grasp_posture.joint_names = ["gripper_finger1_joint", "gripper_finger2_joint"]
    pos = JointTrajectoryPoint() 
    pos.positions.append(1.0)
    pos.positions.append(1.0)
    
    grasp.grasp_posture.points.append(pos)
    
    grasp.max_contact_force = 0
    
    return grasp
    

def createPickupGoal(group="irb_120", target='box1', grasp_pose=PoseStamped()):
    pug = PickupGoal() 
    pug.target_name = target
    pug.group_name = group
    
    # Create grasp to append
    grsp = createGrasp(grasp_pose)
    #pug.possible_grasps.append(grsp)
    pug.allowed_planning_time = 5.0 
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.allowed_touch_objects.append("box1")
    
    return pug


 
if __name__=='__main__':

	##################################################################################################
	# BLOQUE 1: Stuff inicial
	##################################################################################################
	#INicio el nodo obejtivo1y2 de rospy
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('objetivo1y2')

	rospy.loginfo("Connecting to pickup AS")
    	pickup_ac = actionlib.SimpleActionClient('/pickup', PickupAction)
    	pickup_ac.wait_for_server()
    	rospy.loginfo("Succesfully connected.")

      	#Lanzo la escena y el robot
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
	# Create a scene publisher to push changes to the scene
	scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
	# Create a publisher for displaying gripper poses
	gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
       	#INicializo el move group del brazo y la pinza
        arm = moveit_commander.MoveGroupCommander(GROUP_NAME_ARM)
        gripper = moveit_commander.MoveGroupCommander(GROUP_NAME_GRIPPER)    	
	#CON ESTO obtenemos EL link END-EFFECTOR DEL BRAZO
        eef = arm.get_end_effector_link()	
	arm.allow_replanning(True)
	

	# Para publicar trayectorias en RViz
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)


	# Debug Diego
	print "\n El End effector del brazo detectado es= ", eef
	
        # Estado del robot
        print "\n ============ Printing robot state ============"
        print robot.get_current_state()
        print ""  
	
	# Asigno el frame de referencia
	arm.set_pose_reference_frame(FIXED_FRAME)	
	# Doy 5 segundos para cad aintento
	arm.set_planning_time(5)
	# POngo el limite de numero de intentos
	max_pick_attempts = 1

	# CREO CAJA
	box1_id='box1'	
	#DEfino las dimensiones de la caja
	box1_size=[0.05, 0.05, 0.05]
	# LImpiamos la caja (si existe de ejecuciones previas)
        scene.remove_world_object(box1_id)

	# MUEVO ROBOT A HOME. ABRO PINZA
        rospy.loginfo("Moving arm to HOME")
	move_pose_arm(0,0.4,0,0.4,0,0.6)
	#move_pose_arm(0,0,0,0,0,0)
        rospy.sleep(2)
        rospy.loginfo("Opening gripper")
	#gripper.set_named_target("cerrada")
	#gripper.go()
        move_joint_gripper(0.0)

	raw_input("\n Robot listo. BLOQUE 1 terminado. Press key to continue")

	##################################################################################################
	# BLOQUE 2: Creacion escena y poses
	##################################################################################################
       	

	#Añado la caja a la escena
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = robot.get_planning_frame()
   
        # Añadimos la caja que será cogida (alejada por el momento)
        box1_pose.pose.position.x = 0.3
        box1_pose.pose.position.y = -0.3
        box1_pose.pose.position.z = 0.3
        scene.add_box(box1_id, box1_pose, box1_size)
        rospy.sleep(1)
        
	print "\n La pose del pick creada es= \n", box1_pose
	raw_input("\n BLOQUE 2 terminado. Press key to continue")

	##################################################################################################
	# BLOQUE 3: Grasping con pickup()
	##################################################################################################

    	


	pose_grasp = copy.deepcopy(box1_pose)
    	pose_grasp.pose.position.x -= 0.20
    	pose_grasp.pose.position.y -= 0.05
	#goal = createPickupGoal("robotiq_85", 'box1' , pose_grasp)
	#result= arm.pick(box1_id, grsps)

    	goal = createPickupGoal("irb_120", 'box1' , pose_grasp)
    	rospy.loginfo("Sending goal")
    	pickup_ac.send_goal(goal)
    	rospy.loginfo("Waiting for result")
    	pickup_ac.wait_for_result()
    	result = pickup_ac.get_result()


	
	raw_input("\n Pick realizado. Press key to continue")



	# MUEVO ROBOT CON OBJETO COGIDO
        rospy.loginfo("Moving arm while box is picked")	
	move_pose_arm(0,0.2,0,0.5,-0.25,0.3)
	
	#No se me mueve la pinza
	gripper.set_named_target("cerrada")
	move_joint_gripper(1)
	gripper.go()
        rospy.sleep(4)

	raw_input("\n Elimino la caja. Press key to continue")

	# ELimino cualquier objeto cogido
	scene.remove_attached_object(GRIPPER_FRAME, box1_id)
        rospy.sleep(1)	

        # LImpiamos la caja a coger
        scene.remove_world_object(box1_id)
        rospy.sleep(1)	

	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)

