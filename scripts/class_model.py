#!/usr/bin/python

import numpy as np

from   std_msgs.msg         import Float64

class Model_robot:
    def __init__(self):

        # Parametros de las ruedas
        radius = 0.15

        # Para ruedas derechas
        alpha_r1 = 1.04041
        beta_r1  = 2.10117
        l_r1     = 0.3197035
        
        alpha_r2 = 0
        beta_r2  = 3.14159
        l_r2     = 0.254275

        alpha_r3 = 1.04041
        beta_r3  = 0
        l_r3     = 0.3197035

        # Para ruedas izquierdas
        alpha_l1 = 2.10117
        beta_l1  = 1.04041
        l_l1     = 0.3197035

        alpha_l2 = 3.14159
        beta_l2  = 0
        l_l2     = 0.254575

        alpha_l3 = -2.10117
        beta_l3  = -1.04041
        l_l3     = 0.3197035

        ##Vi=cin*r*vll

        # Definicion matrices J1 y J2
        J1 = np.array(  [ (np.sin(alpha_r1+beta_r1),  -np.cos(alpha_r1+beta_r1), -l_r1*np.cos(beta_r1) ),
                          (np.cos(alpha_r2+beta_r2),   np.sin(alpha_r2+beta_r2),  l_r2*np.sin(beta_r2) ),
                          (np.sin(alpha_r3+beta_r3),  -np.cos(alpha_r3+beta_r3), -l_r3*np.cos(beta_r3) ),
                          (np.sin(alpha_l1+beta_l1),  -np.cos(alpha_l1+beta_l1), -l_l1*np.cos(beta_l1) ),
                          (np.sin(alpha_l2+beta_l2),  -np.cos(alpha_l2+beta_l2), -l_l2*np.cos(beta_l2) ),
                          (np.sin(alpha_l3+beta_l3),  -np.cos(alpha_l3+beta_l3), -l_l3*np.cos(beta_l3) )])

        J2 = np.array([(radius , 0 , 0      , 0     , 0      , 0     ),
                       (0      , 0 , 0      , 0     , 0      , 0     ),
                       (0      , 0 , radius , 0     , 0      , 0     ),
                       (0      , 0 , 0      ,radius , 0      , 0     ),
                       (0      , 0 , 0      , 0     , radius , 0     ),
                       (0      , 0 , 0      , 0     , 0      , radius)])

        # Definir Jacob_inv

        self.Jacob_inv1 = np.matmul ( np.linalg.pinv(J2) , J1 )
        self.Jacob_inv2 = np.matmul ( np.linalg.pinv(J1) , J2 )

    def calcVelWheels (self, vel_x, vel_y, vel_ang):
        vec_Vel = np.array([(vel_x),(vel_y),(vel_ang)]) # Velocidad en el marco del robot
        velWheels = np.matmul(self.Jacob_inv1, vec_Vel)
        return velWheels

    def calcVel (self, vel_r1, vel_r2, vel_r3, vel_l1, vel_l2, vel_l3):
        vec_Wheels = np.array([[vel_r1],[0],[vel_r3],[vel_l1],[vel_l2],[vel_l3]])
        vec_Vel = np.matmul(self.Jacob_inv2,vec_Wheels)
        return vec_Vel