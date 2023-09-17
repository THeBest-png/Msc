#!/usr/bin/env python3

from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander import roscpp_initialize
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



roscpp_initialize(sys.argv)

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )

        scene = moveit_commander.PlanningSceneInterface()

        self.robot = robot
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.box_name = ''
        self.scene = scene

        rospy.sleep(1)
        self.ik_client = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    def ik(self, pose):
        
        # Create a GetIK request message
        ik_request = GetPositionIKRequest()
        jp = self.robot.get_current_state().joint_state.position[:4]
        # Fill in the necessary information
        ik_request.ik_request.group_name = "arm"  # Replace with your robot arm group name
        ik_request.ik_request.robot_state.joint_state.name = ["joint1", "joint2", "joint3", "joint4"]  # Add all joint names
        ik_request.ik_request.robot_state.joint_state.position = jp  # Add initial joint positions
        ik_request.ik_request.pose_stamped.header.frame_id = "base_footprint"  # The frame of reference for the end-effector pose
        ik_request.ik_request.pose_stamped.pose.position.x = pose.x  # Desired end-effector x position
        ik_request.ik_request.pose_stamped.pose.position.y = pose.y  # Desired end-effector y position
        ik_request.ik_request.pose_stamped.pose.position.z = pose.z  # Desired end-effector z position
        # Call the service to compute the IK
        try:
            response = self.ik_client(ik_request)
            if response.error_code.val == response.error_code.SUCCESS:
                # Inverse kinematics computed successfully
                # The joint positions can be obtained from the response
                joint_positions = response.solution.joint_state.position
                rospy.loginfo("Inverse Kinematics Solution: {}".format(joint_positions))
                return(joint_positions[:4])
            else:
                rospy.logerr("IK computation failed with error code: {}".format(response.error_code.val))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def go_home_state(self):
        move_group = self.move_group
        move_group.set_named_target("home")

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = False

        for i in range(5):  
            success, plan, _, _ = move_group.plan()

            if success:
                self.execute_plan(plan)
                rospy.loginfo("The robot is moving")
                break

            rospy.sleep(0.5)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        move_group.execute(plan, wait=True)

        rospy.sleep(10)
        
    def go_ready_state(self):
        move_group = self.move_group
        move_group.set_named_target("ready")

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = False

        for i in range(5):  
            success, plan, _, _ = move_group.plan()

            if success:
                self.execute_plan(plan)
                rospy.loginfo("The robot is moving")
                break

            rospy.sleep(0.5)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        return success
    


    def get_ee_pose(self):

        return(self.move_group.get_current_pose().pose)
    
    def go_to_pose_goal(self,pose):
        self.go_ready_state()
        move_group = self.move_group

   
        pose_goal = move_group.get_current_pose()
        pose_goal.pose.position.x = pose[0]
        pose_goal.pose.position.y = pose[1]
        pose_goal.pose.position.z = pose[2]
        # pose_goal.pose.orientation.x = 0
        # pose_goal.pose.orientation.y = 0
        # pose_goal.pose.orientation.z = 0
        pose_goal.pose.orientation.w = 1
        goal = self.ik(pose_goal.pose.position)
        
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = round(goal[0],2)
        joint_goal[1] = round(goal[1],2) 
        joint_goal[2] = round(goal[2],2)
        joint_goal[3] = round(goal[3],2)


        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        rospy.sleep(10)

        ## END_SUB_TUTORIAL

        # For testing:
        #current_joints = move_group.get_current_joint_values()
        return True #all_close(joint_goal, current_joints, 0.5)

    def add_box(self, coordinate, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
   
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "camera_depth_optical_frame"
        box_pose.pose.position.x = coordinate[0]  # x-coordinate in meters
        box_pose.pose.position.y = coordinate[1]  # y-coordinate in meters
        box_pose.pose.position.z = coordinate[2]  # z-coordinate in meters

        box_pose.pose.orientation.x = coordinate[3]
        box_pose.pose.orientation.y = coordinate[4]
        box_pose.pose.orientation.z = coordinate[5]
        box_pose.pose.orientation.w = coordinate[6]
        box_name = "Wall"
        scene.add_box(box_name, box_pose, size=(0.5, 0.001, 1.0))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

            # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL


