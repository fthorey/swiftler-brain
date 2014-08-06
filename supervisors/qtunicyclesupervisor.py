from brain.supervisors.unicyclesupervisor import UnicycleSupervisor
from math import degrees, sqrt, cos, sin, pi, log1p, tan, atan2

from PyQt4 import QtCore, QtGui
class QtUnicycleSupervisor(UnicycleSupervisor):

    def drawHeading(self, painter, option=None, widget=None):
        """Draw the heading direction.
        """
        def drawArrow(painter, color, x1, y1, x2, y2, angle=0.5, ratio=0.1):
            """Draw an arrow.
            """
            # Save state
            painter.save()
            # Rotate and scale
            painter.rotate(degrees(atan2(y2-y1,x2-x1)))
            factor = sqrt((x1-x2)**2 + (y1-y2)**2)
            painter.scale(factor, factor)
            # Draw the arrow
            line = QtCore.QLineF(0, 0, 1, 0)
            xe = 1 - ratio
            ye = tan(angle) * ratio
            line1 = QtCore.QLineF(1, 0, xe, ye)
            line2 = QtCore.QLineF(1, 0, xe, -ye)
            painter.setPen(QtCore.Qt.SolidLine)
            painter.setPen(QtGui.QColor(color))
            painter.drawLine(line)
            painter.drawLine(line1)
            painter.drawLine(line2)
            # Restore state
            painter.restore()

        arrow_l = self.info()["wheels"]["baseLength"] * 2

        # # Draw Robot direction
        # drawArrow(painter, "green", 0, 0, arrow_l, 0)

        # Draw GoToGoal direction
        if self.currentController() is self._gtg:
            gtg_angle = self._gtg.getHeadingAngle(self.info())
            drawArrow(painter, "blue", 0, 0, arrow_l, 0)

        # Draw AvoidObstacle direction
        elif self.currentController() is self._avd:
            avd_angle = self._avd.getHeadingAngle(self.info())
            drawArrow(painter, "red", 0, 0, arrow_l, 0)

        # Draw FollowWall direction
        elif self.currentController() is self._fow:
            along_wall = self._fow._along_wall_vector
            to_wall = self._fow._to_wall_vector

            if to_wall is not None:
                to_angle = degrees(atan2(to_wall[1], to_wall[0]))
                drawArrow(painter, "green", 0, 0, to_wall[0], to_wall[1])
            if along_wall is not None:
                along_angle = degrees(atan2(along_wall[1], along_wall[0]))
                painter.translate(to_wall[0], to_wall[1])
                drawArrow(painter, "purple", 0, 0, along_wall[0], along_wall[1])
