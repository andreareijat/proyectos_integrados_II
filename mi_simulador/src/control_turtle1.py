#!/usr/bin/env python

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
    

def scan_and_move():

    msg=Twist()

    if distancia > 1.0:
        mover_robot(msg)
    else:
        msg.linear.x=0
        girar_robot(msg)
    
    msg.linear.x=0



def laser_callback(msg):
    global distancia
    distancia = msg.ranges[int(len(msg.ranges)/2)]
    
    try:
        scan_and_move()
	
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated. ")

#main

rospy.init_node("nodo_control_turtle")
sub=rospy.Subscriber("/base_scan",LaserScan,laser_callback)
pub=rospy.Publisher("/cmd_vel", Twist, queue_size=1)

distancia=6
velocidad=0.3


rospy.spin()