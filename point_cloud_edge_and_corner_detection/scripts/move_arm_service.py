#!/usr/bin/env python

from __future__ import print_function

from point_cloud_edge_and_corner_detection.srv import move_arm, move_armResponse
import rospy
from point_cloud_edge_and_corner_detection.move_arm import MoveGroupPythonInterfaceTutorial
import numpy as np
from tf.transformations import quaternion_from_matrix, euler_matrix, quaternion_matrix


class move_arm_server ():

    def __init__(self) -> None:

        self.turt_arm = MoveGroupPythonInterfaceTutorial()
        s = rospy.Service('move_arm', move_arm, self.handle_move_arm)
        print("Move arm service is online.")
        rospy.spin()


        

    
    def handle_move_arm(self,req):
        turt_arm = self.turt_arm

        wall_point = [req.wall_x, req.wall_y, req.wall_z]

        M = np.empty((4, 4))
        M[3, :] = [0, 0, 0, 1]
        M = quaternion_matrix([0,0,0,1])
        M[:3, 3] = wall_point
        
        roatation_X = euler_matrix(-0.785398,0,0)
        new_M = np.matmul(M,roatation_X)

        roatation_Y = euler_matrix(0,1.57,0)
        new_M = np.matmul(new_M,roatation_Y)

        roatation_Z = euler_matrix(0,0,-0.1)
        new_M = np.matmul(new_M,roatation_Z)

        q = quaternion_from_matrix(new_M)

        

        check = turt_arm.go_to_pose_goal([req.arm_x, req.arm_y, req.arm_z])

        
        return move_armResponse(check)
    


if __name__ == "__main__":
    rospy.init_node('move_arm_server')
    move_arm_server()