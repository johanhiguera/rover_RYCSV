#!/usr/bin/python

import math
import rospy
import math

import tf2_ros
import tf2_msgs.msg

from    geometry_msgs.msg   import TransformStamped
from    std_msgs.msg        import Bool
from    std_msgs.msg        import Float64MultiArray

class TF:  

    def __init__(self):
        #PARAMETROS
        self.theta = - rospy.get_param("/theta")* 3.141592 / 180.0
        self.ds = rospy.get_param("/ds")
        self.f = rospy.get_param("/f")
        self.coordenadas1 = rospy.get_param("/coordenadas")
        self.position = Float64MultiArray()
        self.pos_x=0
        self.pos_y=0
        self.pos_w=0

        self.update_coor()
        self.creacion_tray()
        #rospy.loginfo(self.coordenadas)
        #rospy.loginfo(self.tray)

        #CREACION DE BROADCAST
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=5)
        self.broadcts  = tf2_ros.TransformBroadcaster()
        self.transform = TransformStamped()

        #SUBSCRIPCION AL TOPICO DE COMUNICACION ENTRE TF_NODE Y CONTROLADOR_NODE
        rospy.Subscriber("/next_coord",Bool,self.callback_next)
        rospy.Subscriber("/position",Float64MultiArray,self.callback_position)
        

        rate = rospy.Rate(self.f)

        self.i = 0                              #VARIABLE PARA RECORRER MATRIZ SELF.TRAY (COORDENADAS DE TRAYECTORIA EN EL SISTEMA DEL ROBOT)

        rospy.loginfo("Nodo TF inicio correctamente ")
        
        while (not rospy.is_shutdown()):
            if(self.i < len(self.tray)-1):
                self.update_odom()
                self.update_goal(self.i+1)
                rate.sleep()              
        
    def update_coor(self):                      #FUNCION PARA CAMBIAR LAS COORDENADAS EN TERMINOS DEL SISTEMA DE REFERENCIA DEL ROBOT
        self.coordenadas = []
        for i in range(len(self.coordenadas1)):
            self.coordenadas.append(0)

        for i in range(len(self.coordenadas1)):
            alpha = math.atan2(self.coordenadas1[i][1],self.coordenadas1[i][0])
            r = math.sqrt(self.coordenadas1[i][1]**2+self.coordenadas1[i][0]**2)
            self.coordenadas[i] = (r*math.cos(alpha+self.theta),r*math.sin(alpha+self.theta))

    def creacion_tray(self):                    #FUNCION PARA CREAR Y LLENAR LA MATRIZ SELF.TRAY
        self.tray = []
        a=0

        for j in range(len(self.coordenadas)-1):
            dx = self.coordenadas[j+1][0]-self.coordenadas[j][0]
            dy = self.coordenadas[j+1][1]-self.coordenadas[j][1]
            s = math.sqrt(dx**2+dy**2)
            n = s/self.ds

            if(abs(n - int(n)) < 1E-3):
                k = int(n)
            else:
                k = int(n)+1
            
            for i in range(k):
                self.tray.append(0)

            for b in range(k):
                self.tray[a]=(self.coordenadas[j][0]+b*dx*self.ds/s,self.coordenadas[j][1]+b*dy*self.ds/s)
                a=a+1
        
        self.tray.append(self.coordenadas[len(self.coordenadas)-1])
        rospy.set_param('/num_coor_tray', len(self.tray))

    def callback_next(self,data):               #FUNCION DE INTERRUPION PARA CAMBIAR A LA SIGUIENTE COORDENADA DE TRAYECTORIA
        if self.i < rospy.get_param("/num_coor_tray"):
            self.i = self.i + 1

        #rospy.loginfo("CAMBIO DE COORDENADA")

    def callback_position(self,data): 
        self.position = data 
        self.pos_x=self.position.data[0]
        self.pos_y=self.position.data[1]
        self.pos_w=self.position.data[2]            

    def update_goal(self, i): #FUNCION PARA ACTUALIZAR LA MATRIZ DE TRANSFORMACION ODOM-GOAL PERIODICAMENTE
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "Goal"
        t.transform.translation.x = self.tray[i][0]
        t.transform.translation.y = self.tray[i][1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcts.sendTransform(t)

    def update_odom(self): #FUNCION PARA ACTUALIZAR LA MATRIZ DE TRANSFORMACION ODOM-GOAL PERIODICAMENTE
        to = TransformStamped()
        to.header.frame_id = "base_footprint"
        to.header.stamp = rospy.Time.now()
        to.child_frame_id = "odom"
        to.transform.translation.x = self.pos_x
        to.transform.translation.y = self.pos_y
        to.transform.translation.z = 0.0

        to.transform.rotation.x = 0.0
        to.transform.rotation.y = 0.0
        to.transform.rotation.z = 0.0
        to.transform.rotation.w = 1.0

        self.broadcts.sendTransform(to)   
