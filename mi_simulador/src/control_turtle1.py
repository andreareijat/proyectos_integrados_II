#!/usr/bin/env python

from re import I
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from collections import deque

def girar_robot(msg):

    msg.angular.z=1.6
    rate=rospy.Rate

    for _ in range(2):
        pub.publish(msg)
        rate.sleep()

def mover_robot(msg):

    global pub
    
    msg.linear.x=velocidad
    pub.publish(msg)

def atrapado(msg):

    principio=msg.ranges[0]
    frente=msg.ranges[len(msg.ranges)/2]
    final=msg.ranges[len(msg.ranges)-1]

    return (principio<0.25 and frente<0.25 and final<0.25)

def trayectoria(msg):

    global odom
    vel=Twist()

    stack=deque()

    principio_array=msg.ranges[0:len(msg.ranges)/3]
    frente_array=msg.ranges[len(msg.ranges):(len(msg.ranges)*2/3)]
    final_array=msg.ranges[(len(msg.ranges)*2/3):len(msg.ranges)]
    

    #direccion_avance=comprueba_dead_end()

    for i in principio_array:
        if i < 0.5:
            vel.linear.x=0
            girar_robot(vel)
    for i in frente_array:
        if i < 0.25:
            vel.linear.x=0
            girar_robot(vel)
    for i in final_array:
        if i < 0.5:
            vel.linear.x=0 
            girar_robot(vel)
    
    mover_robot(vel)
    
    vel.linear.x=0

def odom_callback(msg):
    global odom
    odom=msg


def laser_callback(msg):
    global distancia
    distancia = msg.ranges[int(len(msg.ranges)/2)]


#main

rospy.init_node("nodo_control_turtle")
objetivo=2
sub=rospy.Subscriber("/base_scan",LaserScan,laser_callback)
pub=rospy.Publisher("/cmd_vel", Twist, queue_size=1)
sub=rospy.Subscriber("/odom", Odometry, odom_callback)

velocidad=0.3
contador=0

try:
    trayectoria()
    contador+=1
    if contador==1:
        vector_director_inicial=[posicion_x,posicion_y]

    if contador>1:
        vector_director_actual=[posicion_x,posicion_y]
        
        siguiente=
        anterior=siguiente
    direccion=[]
    direccion.append(vector_director)



except rospy.ROSInterruptException:
    rospy.loginfo("node terminated. ")

rospy.spin()