#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <bhampick/boxpose.h>
#include <moveit_msgs/PickupAction.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <bhampick/tf.h>
#include <tf/transform_listener.h>
#include <gpd_ros/GraspConfig.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
double xx;
double yy;
double px;
double py;
double pz;
double proll;
double ppitch;
double pyaw;

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface &move_group, double x, double y, double z, double roll, double pitch, double yaw, double l, double w, double d)
{
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  // ROS_INFO("yaw = %f", yaw);

  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY((-tau / 4)+roll, (-tau / 8)+pitch, (-tau / 4)+yaw);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  grasps[0].grasp_pose.pose.position.x = (x - (0.085 * cos(yaw)));
  grasps[0].grasp_pose.pose.position.y = (y - (0.085 * sin(yaw)));
  grasps[0].grasp_pose.pose.position.z = z;

  if(abs(y) > abs(x)){
    if(y > 0){
  ROS_INFO("abs(y)>abs(x), y>0");
  orientation.setRPY((-tau / 4)+roll, (-tau / 8)+pitch, (-tau / 4)+yaw+ tau/4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  grasps[0].grasp_pose.pose.position.x = (x - (0.085 * cos(yaw)));
  grasps[0].grasp_pose.pose.position.y = (y - (0.085 * sin(yaw)));
  grasps[0].grasp_pose.pose.position.z = z;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = cos(yaw);
  grasps[0].pre_grasp_approach.direction.vector.y = sin(yaw);
  grasps[0].pre_grasp_approach.direction.vector.z = 0;
  }
  else{
  ROS_INFO("abs(y)>abs(x), y<=0");
  orientation.setRPY((-tau / 4)+roll, (-tau / 8)+pitch, (-tau / 4) + yaw + ((3*tau) / 2));
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  grasps[0].grasp_pose.pose.position.x = (x + (0.085 * cos(yaw)));
  grasps[0].grasp_pose.pose.position.y = (y + (0.085 * sin(yaw)));
  grasps[0].grasp_pose.pose.position.z = z;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = -cos(yaw);
  grasps[0].pre_grasp_approach.direction.vector.y = -sin(yaw);
  grasps[0].pre_grasp_approach.direction.vector.z = 0;
  }
  }
  else{
    if(x > 0){
  ROS_INFO("x > 0");
  orientation.setRPY((-tau / 4)+roll, (-tau / 8)+pitch, (-tau / 4)+yaw);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  grasps[0].grasp_pose.pose.position.x = (x - (0.085 * cos(yaw)));
  grasps[0].grasp_pose.pose.position.y = (y - (0.085 * sin(yaw)));
  grasps[0].grasp_pose.pose.position.z = z;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = cos(yaw);
  grasps[0].pre_grasp_approach.direction.vector.y = sin(yaw);
  grasps[0].pre_grasp_approach.direction.vector.z = 0;
  }
  else{
  ROS_INFO("x <= 0");
  orientation.setRPY((-tau / 4)+roll, (-tau / 8)+pitch, (-tau / 4) + yaw + (tau / 2));
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

  grasps[0].grasp_pose.pose.position.x = (x + (0.085 * cos(yaw)));
  grasps[0].grasp_pose.pose.position.y = (y + (0.085 * sin(yaw)));
  grasps[0].grasp_pose.pose.position.z = z;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = -cos(yaw);
  grasps[0].pre_grasp_approach.direction.vector.y = -sin(yaw);
  grasps[0].pre_grasp_approach.direction.vector.z = 0;
  }
  }

  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);
  ROS_INFO_STREAM("End pick but not return yet");
  return;
}

void createCollisionObjectFrame(tf2_ros::StaticTransformBroadcaster& broadcaster,
                                const std::string& parent_frame,
                                const std::string& child_frame,
                                const double x, const double y, const double z,
                                const double roll, const double pitch, const double yaw)
{
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  transform.transform.rotation = tf2::toMsg(quat);
  broadcaster.sendTransform(transform);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, double x, double y, double z, double roll, double pitch, double yaw, double l, double w, double d)
{ 
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].id = "object";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = w;
  collision_objects[0].primitives[0].dimensions[1] = l;
  collision_objects[0].primitives[0].dimensions[2] = d;

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = x;
  collision_objects[0].primitive_poses[0].position.y = y;
  collision_objects[0].primitive_poses[0].position.z = z;

  tf2::Quaternion boxorientation;
  boxorientation.setRPY(roll, pitch, yaw);
  collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(boxorientation);

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void callback(gpd_ros::GraspConfig grasp)
{
  xx = grasp.position.x;
  yy = grasp.position.y;


  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  ros::NodeHandle nh;
  tf2_ros::StaticTransformBroadcaster broadcaster;
  createCollisionObjectFrame(broadcaster, "world", "box_frame", xx, yy, grasp.position.z, grasp.approach.x, grasp.approach.y, grasp.approach.z);
  addCollisionObjects(planning_scene_interface, xx, yy, grasp.position.z, grasp.approach.x, grasp.approach.y, grasp.approach.z, 0.06, 0.06, 0.12);

  ros::ServiceClient client = nh.serviceClient<bhampick::tf>("tf");
  bhampick::tf srv;
  srv.request.x = grasp.position.x;
  srv.request.y = grasp.position.y;
  srv.request.z = grasp.position.z;
  srv.request.roll = grasp.approach.x;
  srv.request.pitch = grasp.approach.y;
  srv.request.yaw = grasp.position.z;

  client.call(srv);


  ros::WallDuration(1.0).sleep();

  tf::TransformListener listener;

  ros::Rate rate(10.0);

  tf::StampedTransform transform;
  try {
    listener.waitForTransform("panda_link8", "box_frame", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("panda_link8", "box_frame", ros::Time(0), transform);

    tf::Vector3 position = transform.getOrigin();
    tf::Quaternion orientation = transform.getRotation();
    tf::Matrix3x3(orientation).getRPY(proll, ppitch, pyaw);

    px = position.getX();
    py = position.getY();
    pz = position.getZ();
  }
  catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  float a = grasp.approach.z;
  float b = tau/4;
  pyaw = fmod(a, b);
  pick(group, grasp.position.x, grasp.position.y, grasp.position.z, grasp.approach.x, grasp.approach.y, pyaw, 0.06, 0.06, 0.12);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/detect_grasps/best_grasp",1000, callback);
  ros::spin();
  return 0;
}