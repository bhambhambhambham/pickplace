#include <ros/ros.h>
#include <bhampick/boxpose.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_service_client");

  ros::NodeHandle nh;
  ros::Duration(4).sleep();

  // Create a service client for the "my_service" service
  ros::ServiceClient client = nh.serviceClient<bhampick::boxpose>("boxpose");

  // Create a service message object
  bhampick::boxpose srv;
  srv.request.x = 0.5;
  srv.request.y = -0.4;
  srv.request.z = 0.5;
  srv.request.roll = 0;
  srv.request.pitch = 0;
  srv.request.yaw = 2.7;
  srv.request.length = 0.02;
  srv.request.width = 0.02;
  srv.request.depth = 0.2;

  if (client.call(srv)) {
    ROS_INFO("Res");
  } else {
    ROS_ERROR("Failed to call service");
    return 1;
  }
  return 0;
}