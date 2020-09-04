#!/usr/bin/env python
# coding=utf-8

from roslib import message
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

import math
import numpy
import tf
import imutils
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
# from localiza_func import image_callback, callback_kinect, update_transform

def image_callback(msg):
    '''
    Esta funcion creara un circulo alrededor del objeto mas grande que encuentre de color verde.
    Gestionada por el subscriber a /camera/rgb/image_raw
    '''
    # Convertimos el mensaje a imagen de OpenCV
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        print(e)

    # Creamos un filtro HSV para la imagen
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #lower_green = numpy.array([50, 50, 177], numpy.uint8)
    #upper_green = numpy.array([84, 150, 255], numpy.uint8)
    # image = imutils.resize(image, width=600)
    #mask = cv2.inRange(hsv, lower_green, upper_green)
    #blurred = cv2.GaussianBlur(image, (11, 11), 0)
    #hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # ponemos los limites del filtro de color verde
    greenLower = (1, 80, 255)
    greenUpper = (100, 255, 255)

    # ponemos los limites del filtro de color azul
    blueLower = (110,50,255)
    blueUpper = (130,255,255)

    # ponemos los limites del filtro de color rojo
    redLower = (0, 50, 255)
    redUpper = (50, 255, 255)


    if color == 'verde':
        # Erosionamos y dilatamos para mejorar los bordes del filtro verde
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

    elif color == 'azul':
        # Erosionamos y dilatamos para mejorar los bordes del filtro azul
        mask = cv2.inRange(hsv, blueLower, blueUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

    elif color == 'rojo':
        # Erosionamos y dilatamos para mejorar los bordes del filtro rojo
        mask = cv2.inRange(hsv, redLower, redUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)


    # Encontramos todos los contornos
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    contourLength = len(contours)
    
    #dibujo los contornos de las formas
    cv2.drawContours(image,contours,-1,(40,150,255), 2)


    # Comprobamos si algun objeto ha sido encontrado
    if contourLength < 1:
        print "No objects found"      
        sys.exit("No objects found") 
        
    # Escogemos el objeto con mayor area
    area = [0.0]*len(contours)
    for i in range(contourLength):
        area[i] = cv2.contourArea(contours[i])
    ball_image = contours[area.index(max(area))]

    # Rodeamos con un circulo la bola mas grande
    (posu ,posv), radius = cv2.minEnclosingCircle(ball_image)
    ball_center = (int(posu),int(posv))
    ball_radius = int(radius)
    cv2.circle(image, ball_center, ball_radius, (40,150,255), 2)
    cv2.circle(image, ball_center, 5, (40,150,255), -1)

    # Guardamos en el servidor de parametros las posiciones x e y del objeto.
    rospy.set_param('posu', posu)
    rospy.set_param('posv', posv)
    rospy.set_param('radio', ball_radius)

    # Mostramos la imagen en la ventana
    cv2.imshow("TFM_IvanJuez", image)
    cv2.waitKey(3)     


def callback_kinect(data):
    '''
    Funcion callback. Se ejecuta cada vez que llega un mensaje PointCloud. 
    Gestionada por el Subscriber de /camera/depth/points.
    '''
    # Coordenadas centrales recogidas del servidor de parametros
    u_imagen = int(rospy.get_param("posu"))
    v_imagen = int(rospy.get_param("posv"))

    # Llamada a funcion que extrae coordenadas espaciales
    punto_imagen = coordenadas(u_imagen, v_imagen, data)


def coordenadas(u, v, data):
    '''
    Extraccion (x,y,z) desde (u,v) de imagen y data (pointcloud).
    Se llama desde callback_kinect.
    '''
    # Si las coordenadas de imagen estan fuera de rango devolvemos -1
    if (u >= data.height) or (v >= data.width):
        return -1

    # Se usa el metodo read_points para extraer x,y,z
    data_out = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=False, uvs=[[u, v]])
    valores_xyz = next(data_out)
    print "Para (u,v)=", u, v, " (x,y,z)=", valores_xyz

    # llamamos a la funcion que actualiza la posicion del objeto
    valores_frame = valores_xyz
    print ('La posicion nueva es', valores_frame)
    update_transform(valores_frame)
    return valores_xyz

def update_transform(position):
    '''
    Esta funcion actualiza la transformada del objeto a partir de la posicion obtenida por la funcion coordenadas.
    Ademas guardara los datos en el servidor de parametros para que los pueda usar el proceso de pick and place
    '''
    # Actualizamos la transformada
    pos_publisher.sendTransform((position[0],
                                position[1],
                                position[2]), tf.transformations.quaternion_from_euler(0, 0, 0),
                                rospy.Time.now(), "objeto", "camera_rgb_optical_frame")
    (trans, _) = listener.lookupTransform('base_link', 'objeto', rospy.Time(0))
    rospy.loginfo("La transformada del objeto se encuentra en: %f(x), %f(y), %f(z)", trans[0], trans[1], trans[2])
    # Guardamos los datos de la posicion en el servidor de parametros.
    rospy.set_param('posx', trans[0])
    rospy.set_param('posy', trans[1])
    rospy.set_param('posz', trans[2])

if __name__ == '__main__':
    try:
        # Iniciamos el nodo que manejara el procesado de datos
        rospy.init_node('process_cloudpoints', anonymous=True)

        # iniciamos cvbridge para el proceso de transformadas
        bridge = CvBridge()

        # Creamos los subscribers, publishers y el listener
        pos_publisher = tf.TransformBroadcaster()
        listener = tf.TransformListener()
        pos_publisher.sendTransform((0.0, 0.0, 0.0),
                                    (0.0, 0.0, 0.0, 0.0),
                                    rospy.Time.now(),
                                    "objeto","camera_rgb_optical_frame") # creamos el frame del objeto

        # Pongo el input para hacer la máscara del filtro
        color = raw_input("\n Introduzca el color de la pieza : ") 

        print "\n el color elegido es= ", color

        rospy.sleep(0.5)

        rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback, queue_size=1)
        rospy.Subscriber("/camera/depth/points", PointCloud2, callback_kinect)
        
	rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
