#!/usr/bin/env python
import roslib; roslib.load_manifest('point_cloud2_assembler')
import rospy; from laser_assembler.srv import *
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import numpy as np

rospy.init_node("assemble_client")
rospy.wait_for_service("assemble_scans")
try:
    assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
    resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
    print("Got cloud with %u points" % len(resp.cloud.points))
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
    
cloud_points = list(pc2.read_points(resp.cloud, skip_nans=False, field_names = ("x", "y", "z")))
            
#print("running")
pub = rospy.Publisher('point_cloud_data2', Float32MultiArray, queue_size=10)
Floats = Float32MultiArray()

Floats.data = []
for i in range(0,len(cloud_points)):
    #print(i[2])
    if np.isposinf(cloud_points[i][0]):
        pass
    elif np.isneginf(cloud_points[i][0]):
        pass
    else:
        Floats.data.append(cloud_points[i][0])
        Floats.data.append(cloud_points[i][1])
        Floats.data.append(cloud_points[i][2])


pub.publish(Floats)