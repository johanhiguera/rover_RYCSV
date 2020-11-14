#!/usr/bin/python

import rospy
from class_TF import TF

# Init of program
if __name__ == '__main__':

    rospy.init_node('TF_node', anonymous=True)
    rospy.loginfo("Creando nodo TF...")
    TF()
    rospy.spin()