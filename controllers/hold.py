#!/usr/bin/python
# coding: utf-8

from brain.controllers.controller import Controller
import numpy as np

class Hold(Controller):
    """The Hold class hold the robots at its current position.
    """

    def __init__(self, info_):

        # Call PIDController constructor
        super(Hold, self).__init__()

    def execute(self, info_, dt_):
        return 0, 0

    def restart(self, ):
        pass
