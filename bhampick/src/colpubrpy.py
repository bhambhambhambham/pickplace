#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def pubs():
    while not rospy.is_shutdown():
        trans = PoseStamped()
        trans.header.frame_id = "world"
        euler = input().split()
        
        roll = float(euler[0])
        pitch = float(euler[1])
        yaw = float(euler[2])
        
        quat = quaternion_from_euler(roll, pitch, yaw)
        trans.pose.orientation.x = quat[0]
        trans.pose.orientation.y = quat[1]
        trans.pose.orientation.z = quat[2]
        trans.pose.orientation.w = quat[3]
        trans.pose.position.x = 0.37
        trans.pose.position.y = 0
        trans.pose.position.z = 0.2
        pub.publish(trans)
    return
if __name__ == "__main__":
    rospy.init_node("pubrpy")
    global pub
    pub = rospy.Publisher('/custompose', PoseStamped, queue_size = 10)
    rospy.sleep(1)
    pubs()