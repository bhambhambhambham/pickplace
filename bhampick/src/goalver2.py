#!/usr/bin/env python3

import rospy
import tf.transformations as trans
from gpd_ros.msg import GraspConfigList, GraspConfig
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from gpd_ros.msg import GraspConfig

move_group = moveit_commander.MoveGroupCommander("panda_arm")

def cb (grasp_list:GraspConfigList):
    g_list = grasp_list.grasps
    best:GraspConfig   = g_list[0]
    position = best.position
    # rotation_matrix = [best.approach,best.binormal,best.axis]
    rotation_matrix = [best.approach.x,best.approach.y,best.approach.z]
    quantanion = trans.quaternion_from_matrix(rotation_matrix)

    p = Pose()
    p.position = position
    p.orientation = quantanion
    print(p)

    move_group.set_pose_target(p)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    return

if __name__ == "__main__":
    rospy.init_node("goalver2")
    sub = rospy.Subscriber("/detect_grasps/clustered_grasps",GraspConfigList,cb,queue_size=1)
    rospy.spin()