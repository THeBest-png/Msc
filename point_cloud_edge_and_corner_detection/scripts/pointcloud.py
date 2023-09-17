
from point_cloud_edge_and_corner_detection.main_paper_edge import Edge
from point_cloud_edge_and_corner_detection.conversion import Conversion
from point_cloud_edge_and_corner_detection.transform_frames import TransformFrames

#test
import numpy as np
from skspatial.objects import Line, Points
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from point_cloud_edge_and_corner_detection.srv import edge_point,edge_pointResponse
from sensor_msgs.msg import PointCloud2
import rospy

class Point_cloud():
    def __init__(self) -> None:

        
        pub_pcd_topic = "valid_edge_cloud"
        #pub_edge_topic = "edge_pcd"
        
        self.convert = Conversion()

        self.trransform = TransformFrames()
    

        self.pub_pcd = rospy.Publisher(pub_pcd_topic, PointCloud2, queue_size=1)

        self.marker_publisher = rospy.Publisher('Bestfit_points_mark', Marker, queue_size=10)

        self.line_publisher = rospy.Publisher('lines', MarkerArray, queue_size=10)

        self.edge_point = None
        self.wall_center = None
        self.callback()


        


    def callback(self):
        sub_topic = "/camera/depth/color/points"
        ros_cloud = rospy.wait_for_message(sub_topic, PointCloud2, timeout=5)
        pcd = self.convert.RosToOpen3d(ros_cloud)
        
        edge = Edge()
        edge_points, valid_edge_cloud, wall_centroid = edge.edge_extraction(pcd)
        

        ros_pcd = self.convert.Open3dToRos(valid_edge_cloud)
        self.pub_pcd.publish(ros_pcd)

        target_points = np.array(self.target_point(edge_points))

        print(f"Target options are {target_points[:,2]}")
        select_points = int(input("Selecte the target point:"))

        while select_points not in list(range(len(target_points))):
            print(f"Target options are {target_points}")
            select_points = int(input("Selecte the target point:"))

        #target_points[select_points][0] = target_points[select_points][0] - 0.1
        self.marker([target_points[select_points]])
        self.edge_point = target_points[select_points]
        self.wall_center = wall_centroid.center

    
   
    
    def camera_depth_TF_to_base_footprint(self, point):
        point_stamped_msg = PointStamped()

        # Fill in the point coordinates
        point_stamped_msg.point.x = point[0]
        point_stamped_msg.point.y = point[1]
        point_stamped_msg.point.z = point[2]

        if len(point) > 1:
            point_stamped_msg

        # Set the frame ID
        point_stamped_msg.header.frame_id = 'camera_depth_optical_frame'

        # Set the timestamp (optional, you can omit this if you don't need a specific timestamp)
        point_stamped_msg.header.stamp = rospy.Time.now()

        ros_point = self.trransform.point_transform(point_stamped_msg,'base_footprint')

        point = [ros_point.point.x,ros_point.point.y,ros_point.point.z]

        return point



    
    def target_point(self, data):

        target_point = []
        source_frame = 'link1'
        target_frame = 'camera_depth_optical_frame'

        trans = self.trransform.get_transform(source_frame, target_frame)
            
        # Generate points on the line
        x_points = trans.transform.translation.x
        y_points = trans.transform.translation.y
        z_points = trans.transform.translation.z
        
        for data_points in data:

            line_fit = Line.best_fit(Points(data_points))

            point = line_fit.project_point([x_points, y_points, z_points])

            point = self.camera_depth_TF_to_base_footprint(point)
            target_point.append(point)

        return target_point
    

    def marker(self,data):
        
        # Initialize the marker message
        marker = Marker()
        marker.header.frame_id = "base_footprint"  # Change to the appropriate frame ID
        marker.header.stamp = rospy.Time.now()
        marker.ns = "point_markers"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Set the scale of the points (in meters)
        marker.scale.x = 0.05
        marker.scale.y = 0.05

        # Set the color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Set the points (you can add more points by appending to marker.points)
        for point in data:
            add_point = Point()
            add_point.x = point[0]
            add_point.y = point[1] 
            add_point.z = point[2] 

            marker.points.append(add_point)


        self.marker_publisher.publish(marker)
        rospy.sleep(0.5)

def edge_server():
    rospy.init_node('edge_point_server')
    s = rospy.Service('edge_point', edge_point, handle_edge_point)
    rospy.spin()

def handle_edge_point(req):
    call = Point_cloud()

    point = call.edge_point
    wall = call.wall_center 

    return edge_pointResponse(point[0],point[1],point[2],wall[0],wall[1],wall[2])



# -- Example of usage
if __name__ == "__main__":
    
    edge_server()
    