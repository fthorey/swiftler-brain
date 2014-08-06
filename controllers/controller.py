#!/usr/bin/python
# coding: utf-8

class Controller(object):
    """The Controller class defines a behavior for the supervisor class.
    Any implemention must inherit from this class and implement the
    'execute' method to return a unicycle model output
    """

    def execute(self, info_, dt_):
        """Given a state estimation and elapsed time,
        calculate and return robot motion parameters.
        """
        raise NotImplementedError("Controller.execute")

    def restart(self):
        """Reset the controller to the initial state.
        """
        raise NotImplementedError("Controller.restart")
