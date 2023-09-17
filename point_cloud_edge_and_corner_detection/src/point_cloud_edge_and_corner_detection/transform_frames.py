#!/usr/bin/python3
''' transform_frames.py

    Class to transform point and coordinates between robot frames

    Daniel Morris, April 2020, Nov 2021    
    Copyright 2020, 2021
'''
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, PointStamped, TransformStamped
from std_msgs.msg import Header
import tf2_ros, tf2_geometry_msgs

class TransformFrames():
    def __init__(self):
        ''' Create a buffer of transforms and update it with TransformListener '''
        self.tfBuffer = tf2_ros.Buffer()           # Creates a frame buffer
        tf2_ros.TransformListener(self.tfBuffer)   # TransformListener fills the buffer as background task
    
    def get_transform(self, source_frame: str, target_frame: str, stamp=rospy.Time(0), duration=rospy.Duration(0.2)) -> TransformStamped:
        ''' Lookup transform between source_frame and target_frame from the buffer 
            stamp: time for which to look up transform
        '''
        try:
            trans = self.tfBuffer.lookup_transform(target_frame, source_frame, stamp, duration )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f'Cannot find transformation from {source_frame} to {target_frame}')
            raise Exception(f'Cannot find transformation from {source_frame} to {target_frame}') from e
        return trans     # Type: TransformStamped

    def pose_transform(self, pose_array: PoseArray, target_frame: str='odom') -> PoseArray:
        ''' pose_array: will be transformed to target_frame '''
        trans = self.get_transform( pose_array.header.frame_id, target_frame, pose_array.header.stamp )
        new_header = Header(frame_id=target_frame, stamp=pose_array.header.stamp) 
        pose_array_transformed = PoseArray(header=new_header)
        for pose in pose_array.poses:
            pose_s = PoseStamped(pose=pose, header=pose_array.header)
            pose_t = tf2_geometry_msgs.do_transform_pose(pose_s.pose, trans)
            pose_array_transformed.poses.append(pose_t)
        return pose_array_transformed

    def point_transform(self, point: PointStamped, target_frame: str='odom') -> PointStamped:
        ''' Transform a PointStamped to a new frame '''
        trans = self.get_transform( point.header.frame_id, target_frame, point.header.stamp )
        return tf2_geometry_msgs.do_transform_point(point, trans )

    def get_frame_A_origin_frame_B(self, frame_A: str, frame_B: str, stamp=rospy.Time(0) ) -> PoseStamped:
        ''' Returns the pose of the origin of frame_A in frame_B as a PoseStamped '''
        header = Header(frame_id=frame_A, stamp=stamp)        
        origin_A = Pose(position=Point(0.,0.,0.), orientation=Quaternion(0.,0.,0.,1.))
        origin_A_stamped = PoseStamped( pose=origin_A, header=header )
        pose_frame_B = tf2_geometry_msgs.do_transform_pose(origin_A_stamped, self.get_transform(frame_A, frame_B, stamp))
        return pose_frame_B


