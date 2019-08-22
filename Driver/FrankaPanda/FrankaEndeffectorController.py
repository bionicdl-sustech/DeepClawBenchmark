import sys
import time
import moveit_commander
import os

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

class FrankaEndeffectorController:
    def __init__(self):
        # Hand setting
        self.hand = moveit_commander.MoveGroupCommander("hand")
        self.hand.set_planner_id('RRTConnectkConfigDefault')

        # self.openGripper()

    def execute(self, group, plan):
        timeout = 3
        for i in range(timeout):
            res = group.execute(plan)
            time.sleep(1)
            if not res:
                # if fail, swich robot back to mode 2 so that next move can be executed
                # https://github.com/frankaemika/franka_ros/issues/69
                os.system(
                    "rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal \"{}\"")
            else:
                break

    def openGripper(self, distance=0.025):
        target_position = [distance, distance]
        plan = self.hand.plan(target_position)
        self.execute(self.hand, plan)

    def closeGripper(self, distance=0.005):
        target_position = [distance, distance]
        plan = self.hand.plan(target_position)
        res = self.hand.execute(plan)
        time.sleep(1)
        if not res:
            # if fail, swich robot back to mode 2 so that next move can be executed
            # https://github.com/frankaemika/franka_ros/issues/69
            os.system(
                "rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal \"{}\"")
            return 0
        return 1
