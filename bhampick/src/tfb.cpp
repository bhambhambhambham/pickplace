#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <bhampick/boxpose.h>

bool callback(bhampick::boxpose::Request &req, bhampick::boxpose::Response &res)
{
  ROS_INFO("ok");
  ros::spin();
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "new_frame");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("boxpose", callback);
  ros::spin();
  return 0;
}