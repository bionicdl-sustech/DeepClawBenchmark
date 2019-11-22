import os
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.Configuration import *
from modules.calibration.Calibration3D import *


class Calibration(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(Calibration, self).__init__(perception_system, manipulation_system, is_debug)

    def task_display(self):
        calibration_cfg = readConfiguration("/config/modules/calibration.yaml")
        calibrating3d(self.arm, self.camera, calibration_cfg)
