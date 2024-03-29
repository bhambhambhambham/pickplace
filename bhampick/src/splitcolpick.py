#!/usr/bin/env python3

from sympy import false, true
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
import tf
import copy
from std_srvs.srv import Trigger, TriggerResponse

class cr3pick:
    def __init__(self):
        rospy.init_node("picknode", anonymous=True)
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.tf_listener1 = tf.TransformListener()
        self.tf_listener2 = tf.TransformListener()
        self.tf_listener3 = tf.TransformListener()
        self.cartesian1 = False
        self.cartesian2 = False
        self.s = rospy.Service("pick", Trigger, self.callback)
        self.set_home_walkie(0,0,0,0,0,0)
        self.success = False
        self.subframe1 = 'obj_approach_pose'
        self.subframe2 = 'obj_grasp_pose'
        self.subframe3 = 'obj_lift_pose'
        self.count = 1
        self.check = False
        self.setsuccess = False
        self.grasping_group = "hand"
        self.touch_links = self.robot.get_link_names(group=self.grasping_group)
    
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
        self.setsuccess = False
        while (self.success == False):
            ap = Pose()

            self.tf_listener1.waitForTransform("base_link", self.subframe1, rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener1.lookupTransform("base_link", self.subframe1, rospy.Time(0))
            
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
            
            rospy.loginfo("Attempt "+str(self.count)+": "+str(self.success))
            
            if self.success:
                break
            else:
                self.count += 1
                if self.count == 5 and self.check == False:
                    rospy.loginfo("Try Again")
                    self.success = False
                    self.subframe1 = 'obj_approach_pose'
                    self.subframe2 = 'obj_grasp_pose'
                    self.subframe3 = 'obj_lift_pose'
                    self.count = 1
                    self.set_home_walkie(0,0,0,0,0,2.355)
                    
                elif self.count == 5 and self.check == True:
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
            self.tf_listener2.waitForTransform("base_link", self.subframe2, rospy.Time(), rospy.Duration(1.0))
            (trans2, rot2) = self.tf_listener2.lookupTransform("base_link", self.subframe2, rospy.Time(0))
            
            gp.position.x = trans2[0]
            gp.position.y = trans2[1]
            gp.position.z = trans2[2]
            gp.orientation.x = rot2[0]
            gp.orientation.y = rot2[1]
            gp.orientation.z = rot2[2]
            gp.orientation.w = rot2[3]
            
            cartesianpath = []
            cartesianpath.append(copy.deepcopy(gp))
            (plan,fraction) = self.move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
            self.cartesian1 = self.move_group.execute(plan, wait=True)
            
            self.scene.attach_box("robotiq_85_base_link", "box1", touch_links=self.touch_links)
            rospy.sleep(0.5) # Grasp
            
            self.tf_listener3.waitForTransform("base_link", self.subframe3, rospy.Time(), rospy.Duration(1.0))
            (trans3, rot3) = self.tf_listener3.lookupTransform("base_link", self.subframe3, rospy.Time(0))

            rp.position.x = trans3[0]
            rp.position.y = trans3[1]
            rp.position.z = trans3[2]
            
            rp.orientation.x = rot3[0]
            rp.orientation.y = rot3[1]
            rp.orientation.z = rot3[2]
            rp.orientation.w = rot3[3]
            
            cartesianpath[0] = copy.deepcopy(rp)
            (plan,fraction) = self.move_group.compute_cartesian_path(cartesianpath, 0.01, 0.0)
            self.cartesian2 = (self.move_group.execute(plan, wait=True))
            cartesianpath = []
            
            self.success = False
            self.subframe1 = 'obj_approach_pose'
            self.subframe2 = 'obj_grasp_pose'
            self.subframe3 = 'obj_lift_pose'

            self.set_home_walkie(0,0,0,0,0,0)
            self.check = False
            
            if(self.cartesian1 and self.cartesian2):
                x = TriggerResponse()
                x.success = True
                x.message = "Successfully Grasped"
                self.cartesian1 = False
                self.cartesian2 = False
                return x
            else:
                x = TriggerResponse()
                x.success = False
                x.message = "Cartesian Failed"
                self.cartesian1 = False
                self.cartesian2 = False
                return x
 
        else:
            self.success = False
            self.subframe1 = 'obj_approach_pose'
            self.subframe2 = 'obj_grasp_pose'
            self.subframe3 = 'obj_lift_pose'

            self.set_home_walkie(0,0,0,0,0,0)
            self.check = False
            x = TriggerResponse()
            x.success = False
            x.message = "Attempts of finding grasps failed : No valid grasp"
            self.cartesian1 = False
            self.cartesian2 = False
            return x

if __name__ == "__main__":
    cr3pick = cr3pick()
    rospy.spin()