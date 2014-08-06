#!/usr/bin/python
# coding: utf-8

from math import degrees, sqrt, cos, sin, pi, log1p, tan, atan2
from robots.robot import Robot
from brain.controllers.gotogoal import GoToGoal
from brain.controllers.avoidobstacle import AvoidObstacle
from brain.controllers.followwall import FollowWall
from brain.controllers.hold import Hold
from brain.supervisors.supervisor import Supervisor
from utils.matrix import transformationMatrix
import numpy as np
import json

class UnicycleSupervisor(Supervisor):
    """ UnicycleSupervisor is a class that provides a way to control a Woggle robot.
    The UnicycleSupervisor does not move the robot directly. Instead, the supervisor
    selects a controller to do the work and uses the controller outputs
    to generate the robot inputs.
    """

    def __init__(self, robotInfoFile_, supervisorInfoFile_, planClass_, planInfoFile_):
        # Call parent constructor
        super(UnicycleSupervisor, self,).__init__(planClass_, planInfoFile_, supervisorInfoFile_);

        # Load the properties of the robot from file
        try:
            robotInfo_ = json.loads(open(robotInfoFile_, 'r').read())
        except ValueError:
            robotInfo_ = {}

        # Keep track of the position of the robot
        self._info["pos"] = robotInfo_["pos"]

        # Keep track of the old encoders values
        self._info["encoders"] = {}
        self._info["encoders"]["old_leftTicks"] = robotInfo_["encoders"]["leftTicks"]
        self._info["encoders"]["old_rightTicks"] = robotInfo_["encoders"]["rightTicks"]

        # Keep track of the wheels geometry values
        self._info["wheels"] = robotInfo_["wheels"].copy()

        # Keep track of the sensors informations
        self._info["sensors"] = robotInfo_["sensors"].copy()
        self._info["sensors"]["dist"] = self.getIRDistance(robotInfo_)
        self._info["sensors"]["toCenter"] = robotInfo_["sensors"]["ir"]["toCenter"]

        # Follow wall important information
        self._info["direction"] = "left"

        # Distance from center of robot to extremity of a sensor beam
        self._distMax = self._info["sensors"]["toCenter"] + robotInfo_["sensors"]["ir"]["rmax"]
        self._bestDistance = None

        # Create:
        # - a go-to-goal controller
        # - an avoid-obstacle controller
        # - a follow-wall controller
        # - a hold controller
        self._gtg = self.createController('gotogoal.GoToGoal', self._info)
        self._avd = self.createController('avoidobstacle.AvoidObstacle', self._info)
        self._hld = self.createController('hold.Hold', None)
        self._fow = self.createController('followwall.FollowWall', self._info)

        # Set GoToGoal transition functions
        self.addController(self._gtg,
                           (self.atGoal, self._hld),
                           (self.atWall, self._fow))
        # Set AvoidObstacle transition functions
        self.addController(self._avd,
                           (self.atGoal, self._hld),
                           (self.safe, self._fow))
        # Set Hold transition functions
        self.addController(self._hld,
                           (lambda: not self.atGoal(), self._gtg))
        # Set FollowWall transition functions
        self.addController(self._fow,
                           (self.atGoal, self._hld),
                           (self.unsafe, self._avd),
                           (self.wallCleared, self._gtg))

        # Set current controller to GoToGoal
        self._current = self._gtg

    def wallCleared(self, ):
        """Check if the robot should stop following the wall.
        """
        # Did we make progress?
        if self._toGoal >= self._bestDistance:
            return False

        # Are we far enough from the wall,
        # so that we don't switch back immediately
        if self.isAtWall():
            return False

        # Check if we have a clear shot to the goal
        theta_gtg = self._gtg.getHeadingAngle(self.info())
        dtheta = self._fow.getHeadingAngle(self.info()) - theta_gtg

        if self.info()["direction"] == 'right':
            dtheta = -dtheta

        return sin(dtheta) >= 0 and cos(dtheta) >= 0

    def safe(self, ):
        """Check if the surrounding is safe (i.e. no obstacle too close).
        """
        wallFar = self._distMin > self._distMax*0.6
        # Check which way to go
        if wallFar:
            self.atWall()
        return wallFar

    def unsafe(self, ):
        """Check if the surrounding is unsage (i.e. obstacle too close).
        """
        return self._distMin < self._distMax * 0.5

    def atGoal(self):
        """Check if the distance to goal is small.
        """
        return self._toGoal < self.info()["wheels"]["baseLength"]/2

    def isAtWall(self):
        """Check if the distance to obstacle is small.
        """
        # Detect a wall when it is at 80% of the distance
        # from the center of the robot
        return self._toWall < (self._distMax * 0.8)

    def atWall(self, ):
        """Check if the distance to wall is small and decide a direction.
        """
        wall_close = self.isAtWall()

        # Find the closest detected point
        if wall_close:
            dmin = self._distMax
            angle = 0
            for i, d in enumerate(self.info()["sensors"]["ir"]["dist"]):
                if d < dmin:
                    dmin = d
                    angle = self.info()["sensors"]["ir"]["positions"][i][2]

            # Take the direction that follow the wall
            # on the right side
            if angle >= 0:
                self.info()["direction"] = 'left'
            else:
                self.info()["direction"] = 'right'

        # Save the closest we've been to the goal
        if self._bestDistance is None:
            self._bestDistance = self._toGoal

        if self._toGoal < self._bestDistance:
            self._bestDistance = self._toGoal

        return wall_close

    def getIRDistance(self, robotInfo_):
        """Converts the IR distance readings into a distance in meters.
        """
        # Get the current parameters of the sensor
        readings = robotInfo_["sensors"]["ir"]["readings"]
        rmin = robotInfo_["sensors"]["ir"]["rmin"]
        rmax = robotInfo_["sensors"]["ir"]["rmax"]

        #   Conver the readings to a distance (in m)
        dists = [max( min( (log1p(3960) - log1p(r))/30 + rmin, rmax), rmin) for r in readings]
        return dists

    def processStateInfo(self, robotInfo_):
        """Process the current estimation of the robot state.
        """
        # Get the number of ticks on each wheel since last call
        dtl = robotInfo_["encoders"]["leftTicks"] - self.info()["encoders"]["old_leftTicks"]
        dtr = robotInfo_["encoders"]["rightTicks"] - self.info()["encoders"]["old_rightTicks"]

        # Save the wheel encoder ticks for the next estimate
        self.info()["encoders"]["old_leftTicks"] += dtl
        self.info()["encoders"]["old_rightTicks"] += dtr

        # Get old state estimation (in m and rad)
        x, y, theta = self.info()["pos"]

        # Get robot parameters (in m)
        R = robotInfo_["wheels"]["radius"]
        L = robotInfo_["wheels"]["baseLength"]
        m_per_tick = (2*pi*R) / robotInfo_["encoders"]["ticksPerRev"]

        # distance travelled by left wheel
        dl = dtl*m_per_tick
        # distance travelled by right wheel
        dr = dtr*m_per_tick

        theta_dt = -(dr-dl)/L
        theta_mid = theta + theta_dt/2
        dst = (dr+dl)/2
        x_dt = dst*cos(theta_mid)
        y_dt = dst*sin(theta_mid)

        theta_new = theta + theta_dt
        x_new = x + x_dt
        y_new = y + y_dt

        # Process the state estimation
        self.info()["pos"] = (x_new, y_new, theta_new)

        # Process the sensors readings
        self.info()["sensors"]["ir"]["dist"] = self.getIRDistance(robotInfo_)

        # Calculate the distance from every sensors in the robot frame
        vectors = np.array(
            [np.dot(transformationMatrix(p[0], p[1], p[2]),
                       np.array([d,0,1]))
             for p, d in zip(self.info()["sensors"]["ir"]["positions"],
                             self.info()["sensors"]["ir"]["dist"])])

        self._distMin = min((sqrt(a[0]**2 + a[1]**2) for a in vectors))

        # Update goal
        self.info()["goal"] = self._planner.getGoal()

        # Distance to the goal
        self._toGoal = sqrt((x_new - self.info()["goal"][0])**2 +
                            (y_new - self.info()["goal"][1])**2)

        # Distance to the closest obstacle
        self._toWall = self.info()["sensors"]["toCenter"] + min(self.info()["sensors"]["ir"]["dist"])
