#!/usr/bin/env python

from re import I
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

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


def scan_and_move(msg):

    vel=Twist()

    principio_array=msg.ranges[0:len(msg.ranges)/3]
    principio=msg.ranges[0]
    frente_array=msg.ranges[len(msg.ranges):(len(msg.ranges)*2/3)]
    frente=msg.ranges[len(msg.ranges)/2]
    final_array=msg.ranges[(len(msg.ranges)*2/3):len(msg.ranges)]
    final=msg.ranges[len(msg.ranges)]

    direccion_avance=comprueba_dead_end()

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



def laser_callback(msg):
    global distancia,objetivo
    distancia = msg.ranges[int(len(msg.ranges)/2)]
    objetivo=distancia+2

    try:
        scan_and_move(msg)
	
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated. ")

#main

rospy.init_node("nodo_control_turtle")
objetivo=2
sub=rospy.Subscriber("/base_scan",LaserScan,laser_callback)
pub=rospy.Publisher("/cmd_vel", Twist, queue_size=1)

distancia=6
velocidad=0.3


rospy.spin()