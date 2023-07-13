#!/usr/bin/env python3
# roslaunch hw hw2launch.launch

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from gpd_ros.msg import GraspConfig
from tf.transformations import quaternion_from_euler
import tf
import copy

move_group = moveit_commander.MoveGroupCommander("arm")
scene = moveit_commander.PlanningSceneInterface()

def movetogoal():
    rospy.init_node("planner", anonymous=True)
    # print("callback")
    global pose_goal
    pose_goal = Pose()
    
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
    
    # scene.add_box("box", boxpose, size=(0.075, 0.075, 0.075))

    pose_goal.position.x = float(p[0])
    pose_goal.position.y = float(p[1])
    pose_goal.position.z = float(p[2])
    
    # pose_goal.position.x = 0.45+0.153
    # pose_goal.position.y = 0
    # pose_goal.position.z = 0.507+0.25
    
    q = tf.transformations.quaternion_from_euler(float(p[3]),float(p[4]),float(p[5]))
    # q = tf.transformations.quaternion_from_euler(0,0,-1.5708)
    # - Rotation: in Quaternion [-0.000, 0.000, -0.707, 0.707]

    pose_goal.orientation.x = float(q[0])
    pose_goal.orientation.y = float(q[1])
    pose_goal.orientation.z = float(q[2])
    pose_goal.orientation.w = float(q[3])
    
    # pose_goal.orientation.x = 0
    # pose_goal.orientation.y = 0
    # pose_goal.orientation.z = 0
    # pose_goal.orientation.w = 1
    
    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    # car = Pose()
    # car.position.x = float(p[0])+0.1
    # car.position.y = float(p[1])+0.1
    # car.position.z = float(p[2])+0.1
    
    # car.orientation.x = float(q[0])
    # car.orientation.y = float(q[1])
    # car.orientation.z = float(q[2])
    # car.orientation.w = float(q[3])
    
    # cart = []
    # cart.append(copy.deepcopy(car))
    # (plan,fraction) = move_group.compute_cartesian_path(cart, 0.01, 0.0)
    # move_group.execute(plan, wait=True)
    
    # rospy.sleep(1)
    # cart= []

    # car.position.x = float(p[0])+0.2
    # car.position.y = float(p[1])-0.1
    # car.position.z = float(p[2])+0.2
    
    # car.orientation.x = float(q[0])
    # car.orientation.y = float(q[1])
    # car.orientation.z = float(q[2])
    # car.orientation.w = float(q[3])
    
    # cart.append(copy.deepcopy(car))
    # (plan,fraction) = move_group.compute_cartesian_path(cart, 0.01, 0.0)
    # move_group.execute(plan, wait=True)
    
    # move_group.stop()
    # move_group.clear_pose_targets()
    
    
    rospy.spin()

if __name__ == "__main__":
    global p
    p = input().split()
    movetogoal()
