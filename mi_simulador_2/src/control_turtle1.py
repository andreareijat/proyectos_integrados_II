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


#def centrar():

#def comprobar_si_esta_centrado():

def avanzar(msg):

    tope=0.8
    longitud=len(msg.ranges)
    derecha=min(msg.ranges[0:(longitud//3)])
    frente=min(msg.ranges[longitud//3:(longitud*2//3)-1])
    izquierda=min(msg.ranges[longitud*2//3:longitud])

    vel=Twist()
    
    if(frente > tope): #despejado de frente
        rospy.loginfo("caso 1")
        vel.linear.x=0.3  #luego modificar la velocidad
    
    elif(derecha > tope and frente < tope): #despejado a izq y dcha 
        rospy.loginfo("caso 2")  
        vel.angular.z=-0.8           
    
    elif(izquierda > tope and derecha < tope and frente < tope): #despejado izq
        rospy.loginfo("caso 3")
        vel.angular.z=-0.8

    elif(derecha < tope and izquierda < tope and frente < tope): #NO TIENE SALIDA A IZQ, DCHA Y FRENTE
        rospy.loginfo("caso 4")

        #invertimos el vector director
        vel.angular.z=-0.8

    
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
contador_lista=0

sub=rospy.Subscriber("/base_scan_1",LaserScan,laser_callback)
sub=rospy.Subscriber("/base_pose_ground_truth",Odometry, odom_callback)
pub=rospy.Publisher("/cmd_vel", Twist, queue_size=1)



rospy.spin()