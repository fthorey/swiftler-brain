#!/usr/bin/python
# coding: utf-8

from math import atan2, fabs, log, sin, cos, degrees, pi
from brain.controllers.controller import Controller

class PIDController(Controller):
    """The PIDControlller class implements a PID based control to steer the robot to
    a certain heading direction. The heading is recalculated on every execution
    """

    def __init__(self, info_):

        # Call parent constructor
        super(PIDController, self).__init__()

        # PID gains
        # Proportional
        self._Kp = info_["pid"]["gains"]["Kp"]
        # Integral
        self._Ki = info_["pid"]["gains"]["Ki"]
        # Derivative
        self._Kd = info_["pid"]["gains"]["Kd"]

        # Accumulated error
        self._E_k = 0
        # error step k-1
        self._e_k_1 = 0

    def restart(self, ):
        """Restart the PID coefficients.
        """
        self._E_k = 0
        self._e_k_1 = 0

    def getHeadingAngle(self, info_):
        """Return the heading as an angle"""
        # The vector to follow
        heading = self.getHeading(info_)
        return atan2(heading[1], heading[0])

    def getHeading(self, info_):
        """Get the direction in which the controller wants to move the robot
        as a vector.

        return a numpy array [x, y, z] with z = 1.
        """
        raise NotImplementedError("PIDController.getHeading")

    def execute(self, info_, dt_):
        """ Take an estimation of the current state and a heading goal to
        process the appropriate linear velocity v (m/s) and angular velocity (rad/s) to steers
        the robot toward this goal direction.
        """

        # This is the direction we want to go
        e_k = self.getHeadingAngle(info_)

        # Error for the proportional term
        e_P = e_k

        # Error for the integral term
        # Approximate the integrale using the accumulated error, E_k and the error
        # for this time step
        e_I = self._E_k + e_k*dt_
        e_I = (e_I + pi)%(2*pi) - pi

        # Error for the derivative term
        # Approximate the derivative using the previous error, e_k_1
        e_D = (e_k - self._e_k_1) / dt_

        w = self._Kp*e_P + self._Ki*e_I + self._Kd*e_D # (in rad/s)

        # 4. Save errors for the next time step
        self._E_k = e_I
        self._e_k_1 = e_k

        # velocity control
        v =  0.25/(log(fabs(w)+2)+1)

        return v, w
