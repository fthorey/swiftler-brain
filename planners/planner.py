#!/USSR/bin/python
# coding: utf-8

import json

class Planner(object):
    """The planner class provides goal point to the supervisor which
    oversees the control of a single robot.
    """

    def __init__(self, infoFile_):
        """
        """
        # Load the properties of the robot from file
        try:
            self._info = json.loads(open(infoFile_, 'r').read())
        except ValueError:
            self._info = {}

    def info(self, ):
        """Get the parameters that the current controller needs.
        """
        return self._info

    def getGoal(self, ):
        return self.info()["goal"]

    def execute(self, robotInfo_, dt_):
        """
        """
        # Nothing to do for now.
