#!/usr/bin/python
# coding: utf-8

from math import atan2, fabs, log, sin, cos, degrees, pi, log1p
from brain.controllers.pidcontroller import PIDController
from utils.matrix import transformationMatrix
import numpy as np
from PyQt4 import QtGui, QtCore

class AvoidObstacle(PIDController):
    """The AvoidObstacles class steers the robot to a direction calculated with
    the weighted sum of direction of obstacles detected by each sensors
    """

    def __init__(self, info_):

        # Call PIDController constructor
        super(AvoidObstacle, self).__init__(info_)

    def getHeading(self, info_):
        """Get the direction in which the controller wants to move the robot
        as a vector.

        return a numpy array [x, y, z] with z = 1.
        """
        # Map the sensors distance to the robot's frame of reference
        end_w = np.array(
            [np.dot(transformationMatrix(p[0], p[1], p[2]),
                       np.array([d,0,1]))
             for p, d in zip(info_["sensors"]["ir"]["positions"],
                             info_["sensors"]["ir"]["dist"])])


        # Get the resulting vector in the robot's frame of reference
        sum_w = np.sum(end_w, axis=0)
        return np.array([sum_w[0], sum_w[1], 1])
