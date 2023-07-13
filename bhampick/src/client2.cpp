#include "ros/ros.h"
#include <bhampick/posedim.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "placecaller");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<bhampick::posedim>("placeit");
  bhampick::posedim srv;
  srv.request.posercam.header.frame_id = "base_link";
  srv.request.posercam.header.stamp = ros::Time::now();
  srv.request.posercam.pose.position.x = 0.8;
  srv.request.posercam.pose.position.y = 0;
  srv.request.posercam.pose.position.z = 0.9;

  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  srv.request.posercam.pose.orientation.x = q[0];
  srv.request.posercam.pose.orientation.y = q[1];
  srv.request.posercam.pose.orientation.z = q[2];
  srv.request.posercam.pose.orientation.w = q[3];

  srv.request.dimension.x = 0.0264;
  srv.request.dimension.y = 0.0727;
  srv.request.dimension.z = 0.1363;

  if (client.call(srv))
  {
    ROS_INFO("Place Success : %s", srv.response.graspsuccess ? "true" : "false");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}