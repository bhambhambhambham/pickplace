#include <ros/ros.h>
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <bhampick/posedim.h>
#include <std_srvs/Trigger.h>

class place{
    private:
        tf2_ros::StaticTransformBroadcaster bcplace;
        tf2_ros::StaticTransformBroadcaster bcplaceapproach;
        tf2_ros::StaticTransformBroadcaster bcplacesaferetreat;

        ros::ServiceServer service;
        ros::ServiceClient client;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std_srvs::Trigger srv;
    
    public:
        place(ros::NodeHandle *nodehan){
            service = nodehan->advertiseService("placeit", &place::callback2, this);
            client = nodehan->serviceClient<std_srvs::Trigger>("place");
        }

        bool callback2(bhampick::posedim::Request& req, bhampick::posedim::Response& res){
            ROS_WARN("Service Ran");
            createCollisionObjectFrame(bcplace, req.posercam.header.frame_id, "place_place", req.posercam.pose.position.x, req.posercam.pose.position.y,
                                       req.posercam.pose.position.z, req.posercam.pose.orientation.x, req.posercam.pose.orientation.y,
                                       req.posercam.pose.orientation.z, req.posercam.pose.orientation.w);
            tf::TransformListener listener;
            try{
                ros::Time now = ros::Time::now();
                listener.waitForTransform("cr3_base_link", "place_place", now, ros::Duration(0.3));
                createCollisionObjectFrameRPY(bcplaceapproach, "place_place", "place_approach", -0.05, 0, 0, 0, 0, 0);
                createCollisionObjectFrameRPY(bcplacesaferetreat, "place_place", "place_safe_retreat", -0.1, 0, 0, 0, 0, 0);
                tf::TransformListener listener1;
                try{
                ros::Time now = ros::Time::now();
                listener1.waitForTransform("cr3_base_link", "place_approach", now, ros::Duration(0.3));
                ROS_WARN("Pass listener");
                if(client.call(srv)){
                    if(srv.response.success == true){
                        res.graspsuccess = true;
                        return true;
                    }
                    else{
                        res.graspsuccess = false;
                        return false;
                    }
                }
                else{
                    ROS_ERROR("Service Failed");
                    return false;
                }
                return true;
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
            return true;
        }

        void createCollisionObjectFrame(tf2_ros::StaticTransformBroadcaster& broadcaster,
                                const std::string& parent_frame,
                                const std::string& child_frame,
                                const double x, const double y, const double z,
                                const double qx, const double qy, const double qz, const double qw)
        {
        ROS_INFO("Main Object Frame Created");
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;

        transform.transform.rotation.x = qx;
        transform.transform.rotation.y = qy;
        transform.transform.rotation.z = qz;
        transform.transform.rotation.w = qw;

        broadcaster.sendTransform(transform);
        return;
        }

        void createCollisionObjectFrameRPY(tf2_ros::StaticTransformBroadcaster& broadcaster,
                                const std::string& parent_frame,
                                const std::string& child_frame,
                                const double x, const double y, const double z,
                                const double roll, const double pitch, const double yaw)
        {
        ROS_WARN_STREAM(parent_frame);
        ROS_WARN_STREAM(child_frame);
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        transform.transform.rotation = tf2::toMsg(quat);
        broadcaster.sendTransform(transform);
        return;
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "preplaccenode");
    ros::NodeHandle n;
    place k = place(&n);
    ros::spin();
}