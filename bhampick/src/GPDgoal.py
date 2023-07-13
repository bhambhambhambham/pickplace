#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from gpd_ros.msg import GraspConfigList
import tf
import copy
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

move_group = moveit_commander.MoveGroupCommander("arm")
scene = moveit_commander.PlanningSceneInterface()

def pick():
    rospy.init_node("goal", anonymous=True)
    rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, callback)
    rospy.spin()

def callback(data:GraspConfigList):
    ap = Pose()
    gp = Pose()
    rp = Pose()
    
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform("obj_lift", "obj_approach", rospy.Time(0), rospy.Duration(1.0))
    (trans, rot) = tf_listener.lookupTransform("world", "obj_approach", rospy.Time(0))

    ap.position.x = trans[0]
    ap.position.y = trans[1]
    ap.position.z = trans[2]
    ap.orientation.x = rot[0]
    ap.orientation.y = rot[1]
    ap.orientation.z = rot[2]
    ap.orientation.w = rot[3]

    move_group.set_pose_target(ap)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(0.2)
    
    if success:
        (trans2, rot2) = tf_listener.lookupTransform("world", "obj_grasp", rospy.Time(0))
        
        gp.position.x = trans2[0]
        gp.position.y = trans2[1]
        gp.position.z = trans2[2]
        gp.orientation.x = rot2[0]
        gp.orientation.y = rot2[1]
        gp.orientation.z = rot2[2]
        gp.orientation.w = rot2[3]
        
        cartesianpath = []
        cartesianpath.append(copy.deepcopy(gp))
        (plan,fraction) = move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
        move_group.execute(plan, wait=True)
        rospy.sleep(0.7) # Grasp
        
        (trans3, rot3) = tf_listener.lookupTransform("world", "obj_retreat", rospy.Time(0))

        rp.position.x = trans3[0]
        rp.position.y = trans3[1]
        rp.position.z = trans3[2]
        
        rp.orientation.x = rot3[0]
        rp.orientation.y = rot3[1]
        rp.orientation.z = rot3[2]
        rp.orientation.w = rot3[3]
        
        cartesianpath[0] = copy.deepcopy(rp)
        (plan,fraction) = move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
        move_group.execute(plan, wait=True)
    return

if __name__ == "__main__":
    pick()
    
# boxpose = PoseStamped()
    # q = tf.transformations.quaternion_from_euler(0, 0, 0)
    
    # boxpose.header.frame_id = "world"
    # boxpose.pose.position.x = 0.3
    # boxpose.pose.position.y = 0.3
    # boxpose.pose.position.z = 0.3
    
    # boxpose.pose.orientation.x = q[0]
    # boxpose.pose.orientation.y = q[1]
    # boxpose.pose.orientation.z = q[2]
    # boxpose.pose.orientation.w = q[3]
    
    # scene.add_box("box", box_pose, size=(0.075, 0.075, 0.075))