#!/usr/bin/python

import rospy
import numpy as np
import sys, select, termios, tty

from   std_msgs.msg         import Float64
from   geometry_msgs.msg    import Twist
from   sensor_msgs.msg      import JointState
from   rospy.numpy_msg      import numpy_msg

from   class_model          import Model_robot

class LECTURE_KEY:  

    def __init__(self):
        self.vel = Twist()

        self.key_timeout = rospy.get_param("~key_timeout", 0.0)
        if self.key_timeout == 0.0:
                self.key_timeout = None

        self.modelo = Model_robot()

        self.nameTopicPub1 = "/left_motor_1/command"
        self.nameTopicPub2 = "/left_motor_1/command"
        self.nameTopicPub3 = "/left_motor_1/command"
        self.nameTopicPub4 = "/right_motor_1/command"
        self.nameTopicPub5 = "/right_motor_2/command"
        self.nameTopicPub6 = "/right_motor_3/command"
        self.nameTopicPub7 = "/vel_ddrobot"

        self.pub1 = rospy.Publisher(self.nameTopicPub1,Float64,queue_size=10)
        self.pub2 = rospy.Publisher(self.nameTopicPub2,Float64,queue_size=10)
        self.pub3 = rospy.Publisher(self.nameTopicPub3,Float64,queue_size=10)
        self.pub4 = rospy.Publisher(self.nameTopicPub4,Float64,queue_size=10)
        self.pub5 = rospy.Publisher(self.nameTopicPub5,Float64,queue_size=10)
        self.pub6 = rospy.Publisher(self.nameTopicPub6,Float64,queue_size=10)
        self.pub7 = rospy.Publisher(self.nameTopicPub7,numpy_msg(Twist),queue_size=10)
        
        rate = rospy.Rate(10)
        self.vel_L = 0
        self.vel_R = 0
        self.quit = False
        self.key = ' '


        while (not rospy.is_shutdown()):
            if(~self.quit):
                self.detectar_key()
            else:
                break
            
            #vel2send = self.modelo.calcVel(vel_R, vel_L)
            #self.vel.linear.x = vel2send[0]
            #self.vel.linear.y = vel2send[1]
            #self.vel.angular.z = vel2send[2]
            #self.pub7.publish(self.vel)
            rate.sleep()
    
    def detectar_key(self):
        
        self.key = getKey(self,self.key_timeout)
        
        if self.key in moveBindings.keys():
            if ((self.key == 'q') | (self.key == 'Q')):
                self.quit = True
            elif (self.key == ' '):
                self.vel_R = moveBindings[self.key][0]
                self.vel_L = moveBindings[self.key][1]
            else:
                self.vel_R = self.vel_R + moveBindings[self.key][0]
                self.vel_L = self.vel_L + moveBindings[self.key][1]
            
            if (self.vel_R >= 29):
                self.vel_R = 29
            elif (self.vel_R <= -29):
                self.vel_R = -29
            
            if (self.vel_L >= 29):
                self.vel_L = 29
            elif (self.vel_L <= -29):
                self.vel_L = -29

        self.pub1.publish(self.vel_L)
        self.pub2.publish(self.vel_L)
        self.pub3.publish(self.vel_L)
        self.pub4.publish(self.vel_R)
        self.pub5.publish(self.vel_R)
        self.pub6.publish(self.vel_R)


settings = termios.tcgetattr(sys.stdin)

def getKey(self,key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

moveBindings = {
        's':(1.0,-1.0),
        'S':(1.0,-1.0),
        'w':(-4.0,1.0),
        'W':(-1.0,1.0),
        'a':(1.0,1.0),
        'A':(1.0,1.0),
        'd':(-1.0,-1.0),
        'D':(-1.0,-1.0),
        ' ':(0.0,0.0),
        'q':(0.0,0.0),
        'Q':(0.0,0.0)
    }