#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu

import rospy
from point_cloud_edge_and_corner_detection.srv import edge_point, move_arm


dist_th = 0.35
tolerance = np.radians(1)
desired_angle = np.radians(90)
class Move():
    def __init__(self) -> None:
        self.initial_orientation = None
        self.final_orientation = None
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.velocity_msg = Twist()
        self.section = {
                        'front': 0,
                        'left': 0,
                        'right': 0,
                        }
        
        self.state = 0

        self.wallfound = False
        self.edge = False

        self.state_dict_ = {
                0: 'Find wall',
                1: 'Turn right',
                2: 'Follow the wall',
                3: 'Stop',
                4: 'Forward' 
                }
        
        rospy.wait_for_service('edge_point')
        rospy.wait_for_service('move_arm')
        rospy.Subscriber('/scan', LaserScan, self.callback_laser)

        rospy.spin()
    

    def callback_laser(self, msg):

        laser_range = np.array(msg.ranges)
        
        section = {
            'front': min(np.concatenate((laser_range[:10],laser_range[-10:]))),
            'left': min(laser_range[90-10:90+10]),
            'right': min(laser_range[270-10:270+10]),
        }

        

        if self.wallfound:
            if self.state_dict_[self.state] == 'Turn right' :
                self.turn_right_90()
                
            elif self.state_dict_[self.state] == 'Forward':
                #input('Press key to continue')
                print("THis is running ")
                while True:
                    if self.edge == False:
                        rospy.wait_for_service('edge_point')
                        rospy.wait_for_service('move_arm')
                        try:
                            s1 = rospy.ServiceProxy('edge_point', edge_point)
                            s2 = rospy.ServiceProxy('move_arm', move_arm)
                            res = s1()
                            
                            resp2 = s2(res.arm_x ,res.arm_y ,res.arm_z ,res.wall_x ,res.wall_y ,res.wall_z )
                            
                            print(resp2)
                            if resp2.called:
                                #self.edge = True
                                pass
                        except rospy.ServiceException as e:
                            print("Service call failed: %s"%e)

                    
                else:
                    run_duration = rospy.Duration.from_sec(5)  # Replace '10' with the desired time in seconds
                    # Get the current time
                    start_time = rospy.Time.now()
                    # Run the function in a loop until the desired time is reached
                    while rospy.Time.now() - start_time < run_duration:
                        self.move_forward()
                    self.stop()
                    self.state = 3
        else:
            if section['front'] > dist_th:
                rospy.loginfo("Finding wall")
                self.move_forward()

            else:
                self.stop()
                self.state = 1
                self.wallfound = True
        
        

    def move_forward(self):
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = 0
        self.pub.publish(velocity)
    
    def stop(self):
        rospy.loginfo("Stopping")
        velocity = Twist()
        velocity.linear.x = 0
        velocity.angular.z = 0
        self.pub.publish(velocity)
    
    def turn_right_90(self):
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.loginfo("Turning Right 90 degree")

        velocity = Twist()

        # Calculate the angle turned based on IMU data

        angle_turned = self.calculate_angle_difference(self.initial_orientation, self.final_orientation)
    
        
        
        if desired_angle - abs(angle_turned) < tolerance:
            self.stop()
            self.state = 4
        else:
            velocity.linear.x = 0
            velocity.angular.z = -0.3
            self.pub.publish(velocity)

    
    def imu_callback(self, msg):

        if self.initial_orientation is None:
            self.initial_orientation = msg.orientation

        self.final_orientation = msg.orientation

            

        


    def calculate_angle_difference(self, orientation1, orientation2):
        # Convert the orientation from quaternion to Euler angles
        roll1, pitch1, yaw1 = self.euler_from_quaternion(orientation1.x, orientation1.y, orientation1.z, orientation1.w)
        roll2, pitch2, yaw2 = self.euler_from_quaternion(orientation2.x, orientation2.y, orientation2.z, orientation2.w)

        # Calculate the angle difference (in radians) between the two orientations
        angle_difference = yaw2 - yaw1

        # Normalize the angle to the range [-pi, pi]
        if angle_difference > np.pi:
            angle_difference -= 2 * np.pi
        elif angle_difference < -np.pi:
            angle_difference += 2 * np.pi

        return angle_difference

    def euler_from_quaternion(self,x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    




if __name__ == "__main__":
    rospy.init_node('follow_wall')
    Move()
            