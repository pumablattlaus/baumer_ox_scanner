#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from transform_points_from_bag import transform_points_from_bag




class SaveCloudTxt():
    def __init__(self):
        self.txt_file_name = rospy.get_param("~txt_file_name", "/home/robotab/Desktop/line_scanner_pc.txt")
        self.cloud_sub = rospy.Subscriber("cloud_accumulated",PointCloud2, self.pointcloud_cb)
        rospy.loginfo("Save cloud to txt running")

    def pointcloud_cb(self,cloud:PointCloud2):
        # Convert PointCloud2 to an iterable of (x, y, z) points
        gen = pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        # Convert generator to a list of points
        points = np.array(list(gen))
        # Write the accumulated points to a file
        np.savetxt(self.txt_file_name, points, delimiter=',')

if __name__=="__main__":
    rospy.init_node('save_cloud_txt_node')
    SaveCloudTxt()
    rospy.spin()