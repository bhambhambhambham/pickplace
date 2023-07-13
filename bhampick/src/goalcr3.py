#!/usr/bin/env python3
# roslaunch hw hw2launch.launch

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from gpd_ros.msg import GraspConfig
import tf
from geometry_msgs.msg import Quaternion

move_group = moveit_commander.MoveGroupCommander("arm")

def movetogoal():
    rospy.init_node("goal", anonymous=True)
    rospy.Subscriber("/detect_grasps/best_grasp", GraspConfig, callback)
    rospy.spin()

def callback(data:GraspConfig):
    global p
    p = Pose()
    a = -(3.1415926/2)
    b = -(3.1415926/4)

    p.position.x = data.position.x
    p.position.y = data.position.y
    p.position.z = data.position.z

    # q = tf.transformations.quaternion_from_euler(data.axis.x+a, data.axis.y+b, data.axis.z+a)
    # q = tf.transformations.quaternion_from_euler(a, b, a)
    # p.orientation.x = q[0]
    # p.orientation.y = q[1]
    # p.orientation.z = q[2]
    # p.orientation.w = q[3]

    p.orientation.x = data.approach.x
    p.orientation.y = data.approach.y
    p.orientation.z = data.approach.z

    # p.orientation.x = a
    # p.orientation.y = b
    # p.orientation.z = a
    # p.orientation.w = 0
    print("Position : ", p.position.x, p.position.y, p.position.z)
    print("Orientation : ", p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    
    move_group.set_pose_target(p)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    return

if __name__ == "__main__":
    movetogoal()