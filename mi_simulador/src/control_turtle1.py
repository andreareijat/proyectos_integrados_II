#!/usr/bin/env python
from inspect import getinnerframes
from re import I
from socket import gaierror
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import math

def calcular_angulo(vector_actual):
     
    global angulo_vector_actual
    
    try:
        angulo = math.atan(abs(vector_actual[1])/abs(vector_actual[0])) #el resultado ya esta en radianes

        if vector_actual[0] > 0.0 and vector_actual[1]  > 0.0: #PRIMER CUADRANTE
            angulo_vector_actual = angulo
        
        elif vector_actual[0] < 0.0 and vector_actual[1] > 0.0: #SEGUNDO CUADRANTE
            angulo_vector_actual = math.radians(180 - math.degrees(angulo))
        
        elif vector_actual[0] < 0.0 and vector_actual[1] < 0.0: #TERCER CUADRANTE
            angulo_vector_actual = math.radians(180 + math.degrees(angulo))

        elif vector_actual[0] > 0.0 and vector_actual[1] < 0.0: #CUARTO CUADRANTE
            angulo_vector_actual = math.radians(360 - math.degrees(angulo))
    
    except ZeroDivisionError:

        if (vector_actual[0] == 0.0 or vector_actual[0] > 0.0) and vector_actual[1]  == 0.0: 
            angulo_vector_actual=math.radians(0.0)

        if vector_actual[0] == 0.0 and vector_actual[1]  > 0.0: 
            angulo_vector_actual = math.radians(90.0)
        
        if vector_actual[0] < 0.0 and vector_actual[1]  == 0.0: 
            angulo_vector_actual = math.radians(180.0)

        if vector_actual[0] == 0.0 and vector_actual[1] < 0.0: 
            angulo_vector_actual = math.radians(270.0)

    rospy.loginfo("angulo vector actual %f", angulo_vector_actual)

    return angulo_vector_actual

def vector_director(msg):

    global contador_vector

    global posiciones, angulo_vector_actual, angulo_vector_objetivo,vector_objetivo,vector_actual
    
    if contador_vector>1:

        rospy.loginfo("uno")
        vector_actual=[(posiciones[len(posiciones)-1])[0] - (posiciones[len(posiciones)-2])[0], (posiciones[len(posiciones)-1])[1] - (posiciones[len(posiciones)-2])[1]]
        
        angulo_vector_actual=calcular_angulo(vector_actual)
    
    
    if contador_vector==1: #cogerlo solo una vez 
        rospy.loginfo("cero")
    
        vector_objetivo=[(posiciones[len(posiciones)-1])[0] - (posiciones[len(posiciones)-2])[0], (posiciones[len(posiciones)-1])[1] - (posiciones[len(posiciones)-2])[1]]
        
        angulo_vector_objetivo=calcular_angulo(vector_objetivo)
        contador_vector+=1

        vector_actual = vector_objetivo
        angulo_vector_actual = angulo_vector_objetivo
    
    print("vector actual",vector_actual)
    print("vector objetivo", vector_objetivo)

    return angulo_vector_objetivo, angulo_vector_actual

#def centrar():

#def comprobar_si_esta_centrado():


def avanzar(msg):

    tope=0.8
    longitud=len(msg.ranges)
    derecha=min(msg.ranges[0:(longitud//3)])
    frente=min(msg.ranges[longitud//3:(longitud*2//3)-1])
    izquierda=min(msg.ranges[longitud*2//3:longitud])
    #print("derecha",derecha,"izquierda",izquierda,"frente",frente)

    angulo_vector_objetivo,angulo_vector_actual=vector_director(msg)
    rospy.loginfo("angulo objetivo %f, angulo actual %f ", angulo_vector_objetivo, angulo_vector_actual)

    vel=Twist()
    
    if(angulo_vector_objetivo==angulo_vector_actual and frente > tope): #despejado de frente
        rospy.loginfo("caso 1")
        vel.linear.x=0.3  #luego modificar la velocidad
    
    elif((derecha > tope and izquierda > tope and frente < tope) or (derecha > tope and izquierda < tope and frente < tope)): #despejado a izq y dcha o despejado solo dcha
        rospy.loginfo("caso 2")  
        vel.angular.z=-0.8           
    
    elif(izquierda > tope and derecha < tope and frente < tope): #despejado izq
        rospy.loginfo("caso 3")
        vel.angular.z=0.8

    elif(derecha < tope and izquierda < tope and frente < tope): #NO TIENE SALIDA A IZQ, DCHA Y FRENTE
        rospy.loginfo("caso 4")

        #invertimos el vector director
        angulo_vector_objetivo=angulo_vector_objetivo + math.radians(math.degrees(180))
        vel.angular.z=0.8
    
    #elif(angulo_vector_objetivo != angulo_vector_actual and frente > tope and objetivo_libre):

        #girar hacia objetivo

    
    pub.publish(vel)
    vel.linear.x=0.0

def odom_callback(msg):

    global posi_robot, posiciones, contador_lista

    posi_robot=msg
    x=posi_robot.pose.pose.position.x
    y=posi_robot.pose.pose.position.y

    posiciones.append([x,y])
    contador_lista+=1
    
    rospy.loginfo("POSICIONES")
    rospy.loginfo(posiciones)

    if contador_lista > 3:
        posiciones.pop(0)


def laser_callback(msg):

    global distancia
    distancia = msg.ranges[int(len(msg.ranges)/2)]
    rospy.loginfo("nueva vuelta")

    try:
        avanzar(msg)
	
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated. ")



rospy.init_node("nodo_control_turtle")

posi_robot=Odometry()
distancia=6
posiciones=[]
angulo_vector_actual=0.0
angulo_vector_objetivo=0.0
contador_lista=0
contador_vector=1
vector_objetivo=[0,0]
vector_actual=[0,0]

sub=rospy.Subscriber("/base_scan_1",LaserScan,laser_callback)
sub=rospy.Subscriber("/base_pose_ground_truth",Odometry, odom_callback)
pub=rospy.Publisher("/cmd_vel", Twist, queue_size=1)

rospy.spin()