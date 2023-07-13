import rospy
from bhampick.srv import posedim, posedimRequest
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

rospy.init_node('placecaller')

client = rospy.ServiceProxy('placeit', posedim)
srv = posedimRequest()

srv.posercam.header.frame_id = "base_link"
srv.posercam.header.stamp = rospy.Time.now()
srv.posercam.pose.position.x = 0.6
srv.posercam.pose.position.y = 0
srv.posercam.pose.position.z = 0.95

roll = 0
pitch = 0
yaw = 0
quat = quaternion_from_euler(roll, pitch, yaw)

srv.posercam.pose.orientation.x = quat[0]
srv.posercam.pose.orientation.y = quat[1]
srv.posercam.pose.orientation.z = quat[2]
srv.posercam.pose.orientation.w = quat[3]

srv.dimension.x = 0.0727
srv.dimension.y = 0.0727
srv.dimension.z = 0.4

try:
    rospy.wait_for_service('placeit')
    response = client(srv)
    rospy.loginfo("Pick Success: %s", response.graspsuccess)
except rospy.ServiceException as e:
    rospy.logerr("Failed to call service: %s", str(e))
    exit(1)