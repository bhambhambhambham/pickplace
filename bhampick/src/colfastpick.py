#!/usr/bin/env python3

import xdrlib
from sympy import false
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tf
import copy
import math
from bhampick.srv import graspit, graspitResponse

class cr3pick:
    def __init__(self):
        rospy.init_node("pickcol", anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.tf_listener1 = tf.TransformListener()
        self.tf_listener2 = tf.TransformListener()
        self.tf_listener3 = tf.TransformListener()
        self.s = rospy.Service("graspit", graspit, self.callback)
        self.set_home_walkie(0,0,0,0,0,0)
        self.success = False
        self.subframe1 = 'obj_approach_pose'
        self.subframe2 = 'obj_grasp_pose'
        self.subframe3 = 'obj_lift_pose'
        self.count = 1
        self.check = False
        self.setsuccess = False
    
    def set_home_walkie(self,a,b,c,d,e,f):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = a
        joint_goal[1] = b
        joint_goal[2] = c
        joint_goal[3] = d
        joint_goal[4] = e
        joint_goal[5] = f
        self.setsuccess = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        if f == 2.355:
            self.check = True
        return

    def callback(self, data):
        if(data.graspstatus == True):
            pass
        else:
            rospy.loginfo("Grasp Service = False") 
            x = graspitResponse()
            x.successstatus = False
            x.info = "Input service = False"
            return x
        
        while (self.success == False):
            ap = Pose()

            self.tf_listener1.waitForTransform("world", self.subframe2, rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener1.lookupTransform("world", self.subframe2, rospy.Time(0))
            
            ap.position.x = trans[0]
            ap.position.y = trans[1]
            ap.position.z = trans[2]
            ap.orientation.x = rot[0]
            ap.orientation.y = rot[1]
            ap.orientation.z = rot[2]
            ap.orientation.w = rot[3]
        
            self.move_group.set_pose_target(ap)
            self.success = self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(1)
            
            rospy.loginfo("Attempt "+str(self.count)+": "+str(self.success))
            
            if self.success:
                break
            else:
                self.count += 1
                if self.count == 4 and self.check == False:
                    rospy.loginfo("Try Again")
                    self.success = False
                    self.subframe1 = 'obj_approach_pose'
                    self.subframe2 = 'obj_grasp_pose'
                    self.subframe3 = 'obj_lift_pose'
                    self.count = 1
                    self.set_home_walkie(0,0,0,0,0,2.355)
                    
                elif self.count == 4 and self.check == True:
                    rospy.loginfo("Failed 3 Attemps")
                    self.success = False
                    self.subframe1 = 'obj_approach_pose'
                    self.subframe2 = 'obj_grasp_pose'
                    self.subframe3 = 'obj_lift_pose'
                    self.count = 1
                    break

                else:
                    self.subframe1+= 'a'
                    self.subframe2+= 'a'
                    self.subframe3+= 'a'
        
        gp = Pose()
        rp = Pose()
        self.count = 1
        
        if self.success == True:
            grasping_group = "Hand"
            touch_links = self.robot.get_link_names(group=grasping_group)
            self.scene.attach_box("robotiq_85_base_link", "box1", touch_links=touch_links)
            rospy.sleep(0.5) # Grasp
            # self.tf_listener2.waitForTransform("world", self.subframe1, rospy.Time(), rospy.Duration(1.0))
            # (trans2, rot2) = self.tf_listener2.lookupTransform("world", self.subframe2, rospy.Time(0))
            
            # gp.position.x = trans2[0]
            # gp.position.y = trans2[1]
            # gp.position.z = trans2[2]
            # gp.orientation.x = rot2[0]
            # gp.orientation.y = rot2[1]
            # gp.orientation.z = rot2[2]
            # gp.orientation.w = rot2[3]
            
            # cartesianpath = []
            # cartesianpath.append(copy.deepcopy(gp))
            # (plan,fraction) = self.move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
            # self.move_group.execute(plan, wait=True)
            
            # grasping_group = "Hand"
            # touch_links = self.robot.get_link_names(group=grasping_group)
            # self.scene.attach_box("robotiq_85_base_link", "box1", touch_links=touch_links)
            # rospy.sleep(0.5) # Grasp
            
            # self.tf_listener3.waitForTransform("world", self.subframe1, rospy.Time(), rospy.Duration(1.0))
            # (trans3, rot3) = self.tf_listener3.lookupTransform("world", self.subframe3, rospy.Time(0))

            # rp.position.x = trans3[0]
            # rp.position.y = trans3[1]
            # rp.position.z = trans3[2]
            
            # rp.orientation.x = rot3[0]
            # rp.orientation.y = rot3[1]
            # rp.orientation.z = rot3[2]
            # rp.orientation.w = rot3[3]
            
            # cartesianpath[0] = copy.deepcopy(rp)
            # (plan,fraction) = self.move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
            # self.move_group.execute(plan, wait=True)
            # self.scene.remove_attached_object("robotiq_85_base_link", "box1")
            
            self.success = False
            # self.check = False
            self.subframe1 = 'obj_approach_pose'
            self.subframe2 = 'obj_grasp_pose'
            self.subframe3 = 'obj_lift_pose'

            self.set_home_walkie(0,0,0,0,0,0)
            self.scene.remove_attached_object("robotiq_85_base_link", "box1")
            self.check = False
            x = graspitResponse()
            x.successstatus = True
            x.info = "Successfully Grasped"
            return x
        else:
            self.success = False
            # self.check = False
            self.subframe1 = 'obj_approach_pose'
            self.subframe2 = 'obj_grasp_pose'
            self.subframe3 = 'obj_lift_pose'

            self.set_home_walkie(0,0,0,0,0,0)
            self.check = False
            x = graspitResponse()
            x.successstatus = False
            x.info = "Attempts of finding grasps failed : No valid grasp"
            return x

if __name__ == "__main__":
    cr3pick = cr3pick()
    rospy.spin()