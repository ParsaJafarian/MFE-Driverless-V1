import rclpy


import numpy
import array
import time

from mfe_msgs import Cone, ConeDetectionStamped
from sensor_msgs.msg import PointCloud2


class LiDARConeNode(Node):

    def __init__(self):
        super().__init__("lidar_cone_node")

        self.init_params()

        # subscribe to the outputs of the ground removal script
        # should be the pointcloud with the ground removed
        self.create_subscription(PointCloud2, "/lidar/objects", self.callback, 10) # Queue size buffer placeholder

        # publishers - probably point cloud for debugging, then cones
        self.cone_publisher = self.create_publisher(ConeDetectionStamped, "lidar/cone_detection", 1)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, "lidar/cone_points", 10)

    
    def init_params(self):
        # init launch params in here

        return
    
    def create_cone_msg(x, y, z):


        return #msg
    

    def callback(self, msg: PointCloud2):
        # called when pointcloud data is received

        # run some sort of cluster detection
        # clusters -> cones
        # bring back points? 

        # convert cone locs into correct message
        # publish message


        return