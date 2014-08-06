#!/usr/bin/python
# coding: utf-8

from math import degrees, sqrt
from utils import helpers
from brain.planners.planner import Planner
import json

from PyQt4 import QtCore

class Supervisor(object):
    """The supervisor class oversees the control of a single robot.
    """

    def __init__(self, planClass_, planInfoFile_, supervisorInfoFile_):
        # Current controller
        self._current = None

        # Load the properties of the robot from file
        try:
            self._info = json.loads(open(supervisorInfoFile_, 'r').read())
        except ValueError:
            self._info = {}

        # Set current planner
        self._planner = planClass_(planInfoFile_)

        # Dict controller -> (function, controller)
        self._states = {}

        # Set the goal
        self._info["goal"] = self._planner.getGoal()

    def setPlanner(self, planner_):
        """Set the planner that run the robot.
        """
        self._planner = planner_

    def planner(self, ):
        """Get the planner of the robot.
        """
        return self._planner

    def currentController(self, ):
        """Return the current controller.
        """
        return self._current

    def processStateInfo(self, ):
        """Process the current estimation of the state of the robot position.
        """
        raise NotImplementedError("Supervisor.processStateInfo")

    def info(self, ):
        """Get the parameters that the current controller needs.
        """
        return self._info

    def createController(self, moduleString_, info_):
        """Create and return a controller instance for a given controller class.
        """
        controllerClass = helpers.load_by_name(moduleString_, 'brain.controllers')
        return controllerClass(info_)

    def addController(self, controller_, *args):
        """Add a transition table for a state with controller.
        """
        self._states[controller_] = args

    def execute(self, robotInfo_, dt_):
        # Execute planner to update goal if necessary
        self._planner.execute(robotInfo_, dt_)

        # Process state info
        self.processStateInfo(robotInfo_)

        # Switch:
        if self._current in self._states:
            for f, c in self._states[self._current]:
                if f():
                    c.restart()
                    self._current = c
                    print "Switched to {}".format(c.__class__.__name__)
                    break

        #execute the current controller
        return self._current.execute(self.info(), dt_)
