#!/usr/bin/python

import rospy
from class_controlador_polar  import CONTROL

# Init of program
if __name__ == '__main__':

    rospy.init_node('CNTL_node', anonymous=True)
    rospy.loginfo("Creando nodo controlador...")
    CONTROL()  
    rospy.spin()