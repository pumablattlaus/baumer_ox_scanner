#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from math import sin,cos,pi as Pi
from sensor_msgs.msg import JointState
from tf import transformations

from geometry_msgs.msg import PoseStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
import tf
#import roslib; roslib.load_manifest('laser_assembler')


class keyence_transform():
    def __init__(self):
        rospy.init_node('keyence_transform_node')
        self.listener = tf.TransformListener()
        self.base_frame=rospy.get_param("~base_frame", "robot_base_footprint")
        sensor_topic=rospy.get_param("~sensor_topic", "/point_cloud2")
        
        rospy.sleep(1)
        now = rospy.Time.now()
        self.transform = geometry_msgs.msg.TransformStamped()
        #self.listener.waitForTransform("/base_link", "/sensor_optical_frame", now, rospy.Duration(4.0))
        rospy.Subscriber(sensor_topic,PointCloud2, self.pointcloud_cb)   
        self.cloud_pub = rospy.Publisher("/cloud_out",PointCloud2, queue_size = 10)
        self.cloud_out = PointCloud2()
        rospy.loginfo("Transform publisher running")
        rospy.spin()
        

    def pointcloud_cb(self,cloud:PointCloud2):


        self.transform.header.frame_id = self.base_frame
        #self.transform.child_frame_id = "sensor_optical_frame"
        (trans,rot) = self.listener.lookupTransform(self.base_frame, cloud.header.frame_id, rospy.Time())
        self.transform.transform.translation.x = trans[0]
        self.transform.transform.translation.y = trans[1]
        self.transform.transform.translation.z = trans[2]
        self.transform.transform.rotation.x = rot[0]
        self.transform.transform.rotation.y = rot[1]
        self.transform.transform.rotation.z = rot[2]
        self.transform.transform.rotation.w = rot[3]
        
        self.cloud_out = do_transform_cloud(cloud, self.transform)
        self.cloud_pub.publish(self.cloud_out)
                
                





if __name__=="__main__":
    keyence_transform()
