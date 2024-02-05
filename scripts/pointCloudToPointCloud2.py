import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudToPointCloud2:
    def __init__(self, sub_topic='point_cloud', pub_topic='point_cloud2'):
        self.point_cloud_sub = rospy.Subscriber(sub_topic, PointCloud, self.convert_point_cloud)
        self.point_cloud2_pub = rospy.Publisher(pub_topic, PointCloud2, queue_size=10)
        rospy.spin()

    def convert_point_cloud(self, old_point_cloud):
        header = old_point_cloud.header
        points = [(point.x, point.y, point.z) for point in old_point_cloud.points]
        new_point_cloud = pc2.create_cloud_xyz32(header, points)
        self.point_cloud2_pub.publish(new_point_cloud)

if __name__ == '__main__':
    rospy.init_node('point_cloud_to_point_cloud2')
    PointCloudToPointCloud2()