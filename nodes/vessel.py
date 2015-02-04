#!/usr/bin/env python

import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib.patches import Circle

from utils import normalize_angle

class VesselROS(object):
    """ROS wrapper for Vessel model object"""

    def __init__(self, update_rate, is_main_vessel=False, vesseltype='revolt'):
        self.update_rate = update_rate
        self.is_main_vessel = is_main_vessel
        self.vesseltype = vesseltype

        if vesseltype == 'revolt':
            self._scale   = 1.0/20.0
            self._length  = 60.0 * self._scale
            self._breadth = 14.5 * self._scale
            self.model    = VesselModel(x0, h, vesseltype)
            # Vertices of a polygon.
            self._shape = np.asarray([(-self._length/2, -self._breadth/2),
                                      (-self._length/2,  self._breadth/2),
                                      ( self._length/3,  self._breadth/2),
                                      ( self._length/2,  0              ),
                                      ( self._length/3, -self._breadth/2)])
        elif vesseltype == 'viknes':
            self._scale   = 1.0
            self._length  = 8.52 * self._scale
            self._breadth = 2.97 * self._scale
            self.model    = VesselModel(x0, h, vesseltype)
            # Vertices of a polygon.
            self._shape = np.asarray([(-self._length/2, -self._breadth/2),
                                      (-self._length/2,  self._breadth/2),
                                      ( self._length/3,  self._breadth/2),
                                      ( self._length/2,  0              ),
                                      ( self._length/3, -self._breadth/2)])
        else:
            print "Error in selection of vessel! You tried: ", vesseltype
            print "Defaulting to: \'viknes\'"
            self._scale   = 1.0
            self._length  = 8.52 * self._scale
            self._breadth = 2.97 * self._scale
            self.model    = VesselModel(x0, h, 'viknes')

        # Setup ROS specifics


class VesselModel(object):
    """3DOF nonlinear vessel model"""

    def __init__(self, x0, h, vessel_model='viknes'):
        self.x = np.copy(x0)
        self.h = h # Integrator time step

        if vessel_model == 'viknes':
            # Set model parameters
            self.d1u = 16.6
            self.d1v = 9900.0
            self.d1r = 330.0
            self.d2u = 8.25
            self.d2v = 330.0
            self.d2r = 0.0

            self.m   = 3300.0
            self.Iz  = 1320.0

            self.lr  = 4.0
            self.Fxmax = 2310.0
            self.Fymax = 28.8

            self.Kp_p = 0.1
            self.Kp_psi = 5.0
            self.Kd_psi = 1.0
            self.Kp_r   = 8.0

        elif vessel_model == 'hurtigruta':
            # Set model parameters
            self.d1u = 16.6
            self.d1v = 9900.0
            self.d1r = 330.0
            self.d2u = 8.25
            self.d2v = 330.0
            self.d2r = 0.0

            self.m   = 3300.0
            self.Iz  = 1320.0

            self.lr  = 4.0
            self.Fxmax = 2310.0
            self.Fymax = 28.8

            self.Kp_p = 0.1
            self.Kp_psi = 5.0
            self.Kd_psi = 1.0
            self.Kp_r   = 8.0


        # Values other algorithms can use to get information about the model

        # Max yawrate:
        # inertia*r_dot = -(d1r + d2r*|r|)*r + fr_max*lx = 0
        if self.d2r > 0:
            self.est_r_max = 0.5*(-self.d1r + \
                                  np.sqrt(self.d1r**2 + 4*self.d1r*self.Fymax*self.lr)) / d2r
        else:
            self.est_r_max = self.Fymax*self.lr / self.d1r

        # Max yaw acceleration (at r = 0):
        self.est_dr_max = self.Fymax*self.lr / self.Iz

        # Max surge velocity
        # mass*u_dot = -(d1u + d2u*|u|)*u + force_max = 0
        if self.d2u > 0:
            self.est_u_max = 0.5*(-self.d1u + \
                                  np.sqrt(self.d1u**2 + 4*self.d1u*self.Fxmax)) / self.d2u
        else:
            self.est_u_max = self.Fxmax / self.d1u;

        # Min surge velocity (max reverse)
        self.est_u_min = -self.est_u_max;

        # Max surge acceleration
        self.est_du_max = self.Fxmax / self.m
        # Min surge acceleration (max reverse)
        self.est_du_min = -self.est_du_max


    def Cvv(self):
        return np.array([ self.x[4] * self.x[5],
                         -self.x[3] * self.x[5],
                          0                    ])
    def Dvv(self):
        return np.array([self.d2u*self.x[3]*np.abs(self.x[3]) + self.d1u*self.x[3],
                         self.d2v*self.x[4]*np.abs(self.x[4]) + self.d1v*self.x[4],
                         self.d1r*self.x[5]])

    def Tau(self, u_d, psi_d, r_d):
        Fx = (self.d1u + self.d2u*np.abs(self.x[3])) * self.x[3] + \
             (self.x[4]*self.x[5] + self.Kp_p*(u_d - self.x[3])) * self.m

        if psi_d == np.Inf:
            Fy = 1 / self.lr * ( (self.d1r + self.d2r*np.abs(self.x[5]))*self.x[5] + \
                                 self.Iz * self.Kp_r*(r_d - self.x[5]))
        else:
            Fy = self.Kp_psi * self.Iz / self.lr * ((psi_d - self.x[2]) - self.Kd_psi*self.x[5])

        if np.abs(Fx) > self.Fxmax:
            Fx = np.sign(Fx)*self.Fxmax # :todo: Not realistic to go full speed reverse?

        if np.abs(Fy) > self.Fymax:
            Fy = np.sign(Fy)*self.Fymax

        return np.array([Fx, Fy, self.lr*Fy])

    def update(self, u_d, psi_d, r_d):

        self.x[2] = normalize_angle(self.x[2], psi_d)

        Rz = np.array([[ np.cos(self.x[2]),-np.sin(self.x[2]), 0],
                       [ np.sin(self.x[2]), np.cos(self.x[2]), 0],
                       [ 0                , 0                , 1]])

        self.x[0:3] += self.h * np.dot(Rz, self.x[3:6])
        self.x[3:6] += self.h * np.dot(np.diag([1/self.m, 1/self.m, 1/self.Iz]),
                                       self.Tau(u_d, psi_d, r_d) - self.Cvv() - self.Dvv())

        while self.x[2] >= np.pi:
            self.x[2] -= 2*np.pi

        while self.x[2] < -np.pi:
            self.x[2] += 2*np.pi

        return self.x


if __name__ == "__main__":
    pass
