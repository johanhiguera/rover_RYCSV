#!/usr/bin/python

import numpy as np

from   std_msgs.msg         import Float64

class Model_robot:
    def __init__(self):

        # Parametros de las ruedas
        radius = 0.15

        # Para rueda derecha
        alpha_r = - np.pi / 2
        beta_r  = np.pi
        l_r     = 0.15

        # Para rueda izquierda
        alpha_l = np.pi / 2
        beta_l  = 0
        l_l     = 0.15

        ##Vi=cin*r*vll

        # Definicion matrices J1 y J2
        J1 = np.array(  [ (np.sin(alpha_r+beta_r),  -np.cos(alpha_r+beta_r), -l_r*np.cos(beta_r) ),
                          (np.cos(alpha_l+beta_l),   np.sin(alpha_l+beta_l),  l_l*np.sin(beta_l) ),
                          (np.sin(alpha_l+beta_l),  -np.cos(alpha_l+beta_l), -l_l*np.cos(beta_l) ) ])

        J2 = np.array([(radius , 0 , 0),
                       (0      , 0 , 0),
                       (0      , 0 , radius)])

        # Definir Jacob_inv

        self.Jacob_inv = np.matmul ( np.linalg.pinv(J1) , J2 )

    def calcVel (self, vel_r, vel_l):
        vec_Wheels = np.array([[vel_r],[0],[vel_l]])
        vec_Vel = np.matmul(self.Jacob_inv,vec_Wheels)
        return vec_Vel