#!/usr/bin/env python3

import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from visualization_msgs.msg import MarkerArray

def callback(data:MarkerArray):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "zed2i_camera_center"
    t.child_frame_id = "obj"
    t.transform.translation.x = data.markers[0].pose.position.x
    t.transform.translation.y = data.markers[0].pose.position.y
    t.transform.translation.z = data.markers[0].pose.position.z

    t.transform.rotation.x = data.markers[0].pose.orientation.x
    t.transform.rotation.y = data.markers[0].pose.orientation.y
    t.transform.rotation.z = data.markers[0].pose.orientation.z
    t.transform.rotation.w = data.markers[0].pose.orientation.w

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("tfbr")
    rospy.Subscriber("/detect_grasps/plot_grasps", MarkerArray, callback)
    rospy.spin()