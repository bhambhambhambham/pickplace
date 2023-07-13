#!/usr/bin/env python3

import rospy
from gpd_ros.srv import start_gpd, start_gpdResponse, start_gpdRequest
from sensor_msgs.msg import PointCloud2

class PointCloudPublisher:
    def __init__(self):
        rospy.init_node("filt", anonymous=True)
        self.pub = rospy.Publisher("pointcloudsss", PointCloud2, queue_size=10)
        self.sub = rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, self.getpcl)
        self.s = rospy.Service('filt', start_gpd, self.pubbpcl)
    
    def getpcl(self, points):
        self.clouds = points
    
    def pubbpcl(self, data):
        if data.act == 1 and self.pub.get_num_connections() > 0:
            self.pub.publish(self.clouds)
            print("pub")
        return start_gpdResponse(True)

if __name__ == "__main__":
    pcl_publisher = PointCloudPublisher()
    rospy.spin()