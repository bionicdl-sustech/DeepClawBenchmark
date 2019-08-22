import rospy, sys, os
import moveit_commander
from geometry_msgs.msg import Pose
import time
from math import pi
import tf

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

# from Driver.Cobotta.CobottaEndeffectorController import CobottaEndeffectorController

class CobottaController:
    def __init__(self):
        self.HOME_JOINT_VALUES = []
        self.HOME_POSE = [[0.1, 0.1, 0.15], [pi, 0, 0]]
        self.PICK_Z = 0.1
        self.PLACE_Z = 0.1
        self.pose = Pose()

        # ROS node initialization
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface',
                        anonymous=True)

        # Create move group of MoveIt for motion planning
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.calibration_tool = ''
        
        # Arm setting
        #self.group.set_planner_id('RRTConnectkConfigDefault')
        #self.group.set_num_planning_attempts(10)
        #self.group.set_planning_time(5)
        #self.group.set_max_velocity_scaling_factor(0.5)
        
        # Hand setting
        # self.end_effector = CobottaEndeffectorController()
        self.goHome()

    def goHome(self):
        print("homing...")
        self.move(self.HOME_POSE)

    def execute(self, group, plan):
        input = raw_input("wait...")
        res = group.execute(plan)
        time.sleep(0.1)

    def move(self, goal_pose):
        goal_position = goal_pose[0]
        goal_orientation = goal_pose[1]
        q = self.rpy2orientation(goal_orientation[0], goal_orientation[1], goal_orientation[2])
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

        self.pose.position.x = goal_position[0]
        self.pose.position.y = goal_position[1]
        self.pose.position.z = goal_position[2]

        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        self.group.set_pose_target(self.pose)
        plan = self.group.plan()
        # raw_input('wait')
        self.execute(self.group, plan)

    def movej(self,x, y, z, Rx, Ry, Rz, a=0, v=0,useJoint = False):
        if(useJoint==False):
            self.move([[x, y, z], [Rx, Ry, Rz]])
        else:
            plan = self.group.plan([x*3.14159/180.0,y*3.14159/180.0,z*3.14159/180.0,Rx*3.14159/180.0,Ry*3.14159/180.0,Rz*3.14159/180.0])
            self.execute(self.group, plan)

    def openGripper(self):
        self.end_effector.openGripper()

    def closeGripper(self):
        self.end_effector.closeGripper()

    def rpy2orientation(self, row, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(row, pitch, yaw, axes='sxyz')
        return q

    def verifyPosition(self, target_position):
        pass

    def circle_detect(self):
        from ClassicalAlgorithms.CVAlgorithm import CVAlgorithm
        CA = CVAlgorithm()
        realsense = RealsenseController()
        i = 0
        while i <= 5:
            color_image, d, l, r = realsense.getImage()
            i += 1
        gray = CA.color2gray(color_image)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 20, param1=50, param2=30, minRadius=15,
                                   maxRadius=25)
        uvr = []
        if circles is not None:
            circles = np.uint16(np.around(circles))  # shape (1, n, 3)
            for i in range(circles.shape[1]):
                u, v, r = circles[0, i]
                uvr.append([u, v, r])
                cv2.circle(color_image, (u, v), r, (0, 255, 0), 2)
                print(u, v)
        cv2.imshow('c', color_image)
        cv2.waitKey(0)

    def denso_calibration(self):
        ox, oy, oz = 0.13, 0.1, 0.15
        for i in range(2):
            for j in range(2):
                self.move([[ox + i * 0.17, oy - j * 0.25, oz], [pi, 0, 0]])
                # robot.goHome()
                time.sleep(1)

