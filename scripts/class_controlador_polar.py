#!/usr/bin/python

import math
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
import sys, select, termios, tty

import tf2_ros
import tf2_msgs.msg
import tf_conversions

from    geometry_msgs.msg   import Twist
from    nav_msgs.msg        import Odometry
from    std_msgs.msg        import Bool
from    std_msgs.msg        import Float64MultiArray
from    std_msgs.msg        import Float64

class CONTROL:  

    def __init__(self):
        #PARAMETROS
        self.f = rospy.get_param("/f")
        self.vel_cruc = rospy.get_param("/vel_cruc")
        self.w_max = rospy.get_param("/w_max")
        self.Kp = rospy.get_param("/Kp")
        self.num_coor_tray = 0                  #NUMERO DE COORDENADAS DE LA TRAYECTORIA QUE SE CALCULA EN TF_NODE
        self.pos_x=0.0
        self.pos_y=0.0
        self.pos_w=0.0

        #CREACION DEL LISTENER
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #COMUNICACION ENTRE EL CONTROLADOR_NODE Y KOBUKI
        self.pub1 = rospy.Publisher("/vel_order",Float64MultiArray,queue_size=50)
        self.pub2 = rospy.Publisher("/position",Float64MultiArray,queue_size=50)

        #CREACION DEL NODO DE COMUNICACION ENTRE TF_NODE Y CONTROLADOR_NODE
        self.pub_next_coord = rospy.Publisher("/next_coord", Bool, queue_size=1)
        
        #MENSAJES PARA PUBLICACION EN TOPICOS
        self.next = Bool()                      #VARIABLE PARA LA COMUNICACION ENTRE TF_NODE Y CONTROLADOR_NODE
        self.next.data = True
        self.order = Float64MultiArray()
        self.position = Float64MultiArray()
        self.vel_y = 0.0
        self.w = 0.0
        self.theta_parcial=0
        rate = rospy.Rate(self.f)

        self.j=0                                #VARIABLE QUE INDICA CUANTOS CAMBIOS DE COORDENADAS SE HAN REALIZADO
        self.MTH = [0]                          #MATRIZ DE TRANSFORMACION ODOM-GOAL

        while(self.num_coor_tray == 0):
            self.num_coor_tray = rospy.get_param("/num_coor_tray")

        rospy.loginfo("Nodo controlador inicio correctamente ")
        
        while (not rospy.is_shutdown()):
            if(self.j < self.num_coor_tray):
                self.j = self.j + 1
                MTH = [0]
                while(len(self.MTH) == 1):
                    self.MTH = self.Calcular_MTH()
                    rate.sleep()

                self.rho    = 1
                self.theta  = 0
                while (self.rho >= self.vel_cruc/self.Kp[0]):
                    self.MTH = self.Calcular_MTH()
                    if(len(self.MTH) != 1):
                        self.Controlador_polar()
                    
                    rate.sleep()
                self.pub_next_coord.publish(self.next)
                rate.sleep()

    
    def Calcular_MTH(self):                     #FUNCION PARA CALCULAR LA MATRIZ DE TRANSFORMACION ENTRE KOBUKI-GOAL
        try:
            trans_base_marker = self.tfBuffer.lookup_transform("base_footprint", "Goal", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error trying to look for transform")
            return [0]
        quat_from_ROS = np.array([trans_base_marker.transform.rotation.x, \
                                    trans_base_marker.transform.rotation.y, \
                                    trans_base_marker.transform.rotation.z, \
                                    trans_base_marker.transform.rotation.w])
        rt_mat_from_ROS = tf_conversions.transformations.quaternion_matrix(quat_from_ROS)
        MTH_GOAL = rt_mat_from_ROS.copy()
        MTH_GOAL[0,3] = trans_base_marker.transform.translation.x
        MTH_GOAL[1,3] = trans_base_marker.transform.translation.y
        
        return MTH_GOAL
    
    def Controlador_polar(self):                #FUNCION PARA EL CONTROL EN SISTEMA POLAR DEL KOBUKI

        self.dx     =  self.MTH[0,3]
        self.dy     =  self.MTH[1,3]
        self.theta  =  math.atan2(self.MTH[1,0],self.MTH[0,0])
        self.rho    =  math.sqrt(self.dx**2+self.dy**2)
        self.beta   = -math.atan2(self.dy,self.dx)
        self.alpha  = -self.theta - self.beta

        self.vel_y = self.Kp[0]*self.rho
        self.w = self.Kp[1]*self.alpha + self.Kp[2]*self.beta

        if self.vel_y > self.vel_cruc:
            self.vel_y = self.vel_cruc
        elif self.vel_y < -self.vel_cruc:
            self.vel_y = -self.vel_cruc

        if self.w > self.w_max:
            self.w = self.w_max
        elif self.w < -self.w_max:
            self.w = -self.w_max

        self.pos_w=self.pos_w+self.w/(self.f)#*11.8/3.1416
        self.pos_x=self.pos_x+math.sin(self.pos_w)*self.vel_y/(self.f)#2*
        self.pos_y=self.pos_y+math.cos(self.pos_w)*self.vel_y/(self.f)#2*
        
        
        self.order.data = [self.vel_y,self.w]
        self.position.data = [self.pos_x,self.pos_y,self.pos_w]
        self.pub1.publish(self.order)
        self.pub2.publish(self.position)

        rospy.loginfo("A")
        rospy.loginfo(self.dx)
        rospy.loginfo(self.dy)
        rospy.loginfo(self.pos_w)
        rospy.loginfo(self.vel_y)
        rospy.loginfo(self.w)  
        #rospy.loginfo("self.MTH")
        #rospy.loginfo(self.MTH)

    


