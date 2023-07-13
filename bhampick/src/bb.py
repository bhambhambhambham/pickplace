#!/usr/bin/env python3

import enum
import rospy
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
import tf2_ros
import tf2_geometry_msgs
from bhampick.srv import objectlist, objectlistRequest, objectlistResponse

class haiyaa():
    def __init__(self):
        self.tfedobj = []
        self.s = rospy.Service("transformobj", objectlist, self.callback)
        self.srv = objectlistRequest()
        self.client = rospy.ServiceProxy('objects', objectlist)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.count = 0

    def callback(self, data):
        float_list = [round(num, 3) for num in data.objects]
        dat = [float_list[i:i+6] for i in range(0, len(float_list), 6)]
        for i in dat:
            tf_pose = Pose()
            tf_pose.position.x = i[0]
            tf_pose.position.y = i[1]
            tf_pose.position.z = i[2]
            
            tf_pose.orientation.x = 0
            tf_pose.orientation.y = 0
            tf_pose.orientation.z = 0
            tf_pose.orientation.w = 1
            
            dimx = i[3]
            dimy = i[4]
            dimz = i[5]
            # pose of the object related to the camera
            self.transform_pose(tf_pose, "zed2i_1", "cr3_base_link", dimx, dimy, dimz)
            
        self.srv.objects = [item for sublist in self.tfedobj for item in sublist]
        print(self.srv.objects)
        self.createCollisionObjects(self.tfedobj)
        if data.place:
            self.srv.place = True
        else: self.srv.place = False
        self.srv.floor = data.floor
        try:
            rospy.wait_for_service('transformobj')
            response = self.client(self.srv)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call service: %s", str(e))
            exit(1)
        self.tfedobj = []
        x = objectlistResponse()
        return x
    
    def createCollisionObjects(self, objects):
        for index, obj in enumerate(objects):
            boxpose = PoseStamped()
            self.count+=1
            
            boxpose.header.frame_id = "cr3_base_link"
            boxpose.pose.position.x = obj[0]
            boxpose.pose.position.y = obj[1]
            boxpose.pose.position.z = obj[2]
            
            boxpose.pose.orientation.x = 0
            boxpose.pose.orientation.y = 0
            boxpose.pose.orientation.z = 0
            boxpose.pose.orientation.w = 1
            
            name = "Cbox"+str(self.count)
            self.scene.add_box(name, boxpose, size=(obj[3], obj[4], obj[5]))

    def transform_pose(self, input_pose, from_frame, to_frame, x, y, z):

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            self.tfedobj.append([output_pose_stamped.pose.position.x, output_pose_stamped.pose.position.y,
                                 output_pose_stamped.pose.position.z, x, y, z])
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

if __name__ == "__main__":
    rospy.init_node("transform_test")
    k = haiyaa()
    rospy.spin()