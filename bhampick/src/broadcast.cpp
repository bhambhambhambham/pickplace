#include <ros/ros.h>
#include <bhampick/pub.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <bhampick/tf.h>

float xx;
float yy;
float zz;
float roll;
float pitch;
float yaw;

bool msgCallback(bhampick::tf::Request &req, bhampick::tf::Response &res)
{ 
  xx = req.x;
  yy = req.y;
  zz = req.z;
  roll = req.roll;
  pitch = req.pitch;
  yaw  = req.yaw;
  return true;
}
void bcloop(float xa, float ya, float za, float rolla, float pitcha, float yawa){
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(xx, yy, zz));
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.setRotation(q);
  static tf::TransformBroadcaster br;

  ros::Rate rate(100); 
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "box_frame"));
  ROS_INFO("Bced");
  rate.sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_node");
  ros::NodeHandle nh;
  ros::ServiceServer ser = nh.advertiseService("tf", msgCallback);
  bcloop(xx,yy,zz,roll,pitch,yaw);
  ros::spin();

  return 0;
}