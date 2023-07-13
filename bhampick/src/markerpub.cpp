#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "markerpub");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("marker", 1000);

  visualization_msgs::MarkerArray marray;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "obj";
  marker.header.stamp = ros::Time();
  marker.ns = "name";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0.25;
  marker.pose.position.y = 0.25;
  marker.pose.position.z = 0.25;

  tf2::Quaternion quat;
  quat.setRPY(0.2, 0.2, 0.2);

  marker.pose.orientation = tf2::toMsg(quat);

  marker.scale.x = 0.1;
  marker.scale.x = 0.02;
  marker.scale.x = 0.02;

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marray.markers[0] = marker;

  marker_pub.publish(marray);
  ros::spin();
}