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
#include <bhampick/dimmsg.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <bhampick/posedim.h>
#include <std_srvs/Trigger.h>

class tffrompose{
    private:
        tf2_ros::StaticTransformBroadcaster bcmain;
        tf2_ros::StaticTransformBroadcaster bcobj;
        tf2_ros::StaticTransformBroadcaster bc1;
        tf2_ros::StaticTransformBroadcaster bc2;
        tf2_ros::StaticTransformBroadcaster bc3;
        tf2_ros::StaticTransformBroadcaster bc4;
        tf2_ros::StaticTransformBroadcaster bc5;
        tf2_ros::StaticTransformBroadcaster bc6;
        tf2_ros::StaticTransformBroadcaster bc7;
        tf2_ros::StaticTransformBroadcaster bc8;
        tf2_ros::StaticTransformBroadcaster bc9;
        tf2_ros::StaticTransformBroadcaster bc10;
        tf2_ros::StaticTransformBroadcaster bc11;
        tf2_ros::StaticTransformBroadcaster bc12;
        tf2_ros::StaticTransformBroadcaster bcplace;
        tf2_ros::StaticTransformBroadcaster bcplaceapproach;
        tf2_ros::StaticTransformBroadcaster bcplacesaferetreat;

        ros::ServiceServer service;
        ros::ServiceServer service2;
        ros::ServiceClient client;
        ros::ServiceClient client2;
        double ax, ay, az;
        double qx, qy, qz, qw;
        double dimx, dimy, dimz;
        double dimdatax, dimdatay, dimdataz;
        double roll, pitch, yaw;
        bool check = false;
        double gripperlength = 0.085;
        double pi = M_PI;
        tf::Quaternion orientation;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        std_srvs::Trigger srv;
        std_srvs::Trigger srv2;
        tf::StampedTransform tfStamped;
        tf::StampedTransform tfStamped2;
    
    public:
        tffrompose(ros::NodeHandle *nodehan){
            service = nodehan->advertiseService("pickit", &tffrompose::callback, this);
            service2 = nodehan->advertiseService("placeit", &tffrompose::callback2, this);
            client = nodehan->serviceClient<std_srvs::Trigger>("pick");
            client2 = nodehan->serviceClient<std_srvs::Trigger>("place");
        }

        bool callback2(bhampick::posedim::Request& req, bhampick::posedim::Response& res){
            ROS_WARN("Place");
            createCollisionObjectFrame(bcplace, req.posercam.header.frame_id, "place_place", req.posercam.pose.position.x, req.posercam.pose.position.y,
                                       req.posercam.pose.position.z, req.posercam.pose.orientation.x, req.posercam.pose.orientation.y,
                                       req.posercam.pose.orientation.z, req.posercam.pose.orientation.w);
            tf::TransformListener listener;
            try{
                ros::Time now = ros::Time::now();
                listener.waitForTransform("cr3_base_link", "place_place", now, ros::Duration(0.3));
                createCollisionObjectFrameRPY(bcplaceapproach, "place_place", "place_approach", -0.05, 0, 0, 0, 0, 0);
                createCollisionObjectFrameRPY(bcplacesaferetreat, "place_place", "place_safe_retreat", -0.1, 0, 0, 0, 0, 0);
                
                try{
                ros::Time now = ros::Time::now();
                listener.waitForTransform("cr3_base_link", "place_approach", now, ros::Duration(0.3));
                if(client2.call(srv2)){
                    if(srv2.response.success == true){
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

        bool callback(bhampick::posedim::Request& req, bhampick::posedim::Response& res){
            ROS_WARN("Pick");
            createCollisionObjectFrame(bcmain, req.posercam.header.frame_id, "mainobj", req.posercam.pose.position.x, req.posercam.pose.position.y,
                                       req.posercam.pose.position.z, req.posercam.pose.orientation.x, req.posercam.pose.orientation.y,
                                       req.posercam.pose.orientation.z, req.posercam.pose.orientation.w);
            addCollisionObjects(planning_scene_interface, "cam", "box1", req.posercam.pose.position.x, req.posercam.pose.position.y,
                                       req.posercam.pose.position.z, req.posercam.pose.orientation.x, req.posercam.pose.orientation.y,
                                       req.posercam.pose.orientation.z, req.posercam.pose.orientation.w, req.dimension.x, 
                                       req.dimension.y, req.dimension.z);

            tf::TransformListener listener;
            try{
                ros::Time now = ros::Time::now();
                listener.waitForTransform("cr3_base_link", "mainobj", now, ros::Duration(0.3));
                listener.lookupTransform("/cr3_base_link", "mainobj", ros::Time(0), tfStamped);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }

            tf::Vector3 point = tfStamped.getOrigin();
            tf::Quaternion objor = tfStamped.getRotation();

            ax = point[0];
            ay = point[1];
            az = point[2];

            qx = objor[0];
            qy = objor[1];
            qz = objor[2];
            qw = objor[3];

            tf::Quaternion q(qx, qy, qz, qw);
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            dimdatax = req.dimension.x;
            dimdatay = req.dimension.y;
            dimdataz = req.dimension.z;
            
            setdimandyaw(roll, pitch, yaw);
            createGraspCandidate();

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

        void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                                std::string header, std::string name,
                                double ax, double ay, double az, double qx, double qy, double qz, 
                                double qw, double dx, double dy, double dz)
        { 
            ROS_INFO("Adding Collision Object");
            moveit_msgs::CollisionObject collision_object;
            collision_object.header.frame_id = header;
            collision_object.id = name;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = dx;
            primitive.dimensions[1] = dy;
            primitive.dimensions[2] = dz;

            geometry_msgs::Pose box_pose;
            box_pose.orientation.x = qx;
            box_pose.orientation.y = qy;
            box_pose.orientation.z = qz;
            box_pose.orientation.w = qw;
            box_pose.position.x = ax;
            box_pose.position.y = ay;
            box_pose.position.z = az;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;

            std::vector<moveit_msgs::CollisionObject> collision_objects;
            collision_objects.push_back(collision_object);

            ROS_INFO("Object Added");
            planning_scene_interface.addCollisionObjects(collision_objects);
            check = false;
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
        check = true;
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

        void setdimandyaw(const double r, const double p, const double y)
        {
            tf::TransformListener listener2;
            try {
                listener2.waitForTransform("cr3_base_link", "mainobj", ros::Time(0), ros::Duration(1.0));
                }
            catch (tf::TransformException& ex) 
            {
                ROS_ERROR("%s", ex.what());
            }
            
            if((fabs(r)>=1.56 && fabs(r)<=1.58)&&(fabs(p)>=1.56 && fabs(p)<=1.58)){
                dimx = dimdatay;
                dimy = dimdataz;
                dimz = dimdatax;
            }
            else if((fabs(r)>=3.13 && fabs(r)<=3.15)&&(fabs(p)>=1.56 && fabs(p)<=1.58)){
                dimx = dimdataz;
                dimy = dimdatay;
                dimz = dimdatax;
            }
            else if(fabs(r)>=1.56 && fabs(r)<=1.58){
                dimx = dimdatax;
                dimy = dimdataz;
                dimz = dimdatay;
            }
            else if(fabs(p)>=1.56 && fabs(p)<=1.58){
                dimx = dimdataz;
                dimy = dimdatay;
                dimz = dimdatax;
            }
            else{
                dimx = dimdatax;
                dimy = dimdatay;
                dimz = dimdataz;
            }

            if(y > pi/2 || y < -pi/2){
                createCollisionObjectFrameRPY(bcobj, "cr3_base_link", "obj", ax, ay, az, 0, 0, y+pi);
            }
            else{
                createCollisionObjectFrameRPY(bcobj, "cr3_base_link", "obj", ax, ay, az, 0, 0, y);
            }
            return;
        }

        void createGraspCandidate(){
            tf::TransformListener listener;
            try {
                listener.waitForTransform("cr3_base_link", "obj", ros::Time(0), ros::Duration(1.0));
                if(dimx > gripperlength && dimy > gripperlength){
                    ROS_INFO("Cannot grip");
                }

                else if(dimx < gripperlength && dimy < gripperlength){
                    if((ay > 0)&&((ax > fabs(ay))&&((yaw > -pi/2 && yaw < 0)||(yaw > pi/2 && yaw < pi)))){
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", 0, -dimy/2, 0, 0, 0, pi/2);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", 0, -dimy/2-0.05, 0, 0, 0, pi/2);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", 0, -dimy/2, 0.05, 0, 0, pi/2);

                        createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", -dimx/2, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", -dimx/2-0.05, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", -dimx/2, 0, 0.05, 0, 0, 0);

                        createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", dimx/2, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", dimx/2+0.05, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", dimx/2, 0, 0.05, 0, 0, pi);

                        createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, 0, pi/2, pi/2);
                        createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, 0, pi/2, pi/2);
                        createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, 0, pi/2, pi/2);
                    }
                    else if((ay > 0)&&((ax < fabs(ay))&&((yaw > 0 && yaw < pi/2)||(yaw > -pi && yaw < -pi/2)))){
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", 0, -dimy/2, 0, 0, 0, pi/2);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", 0, -dimy/2-0.05, 0, 0, 0, pi/2);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", 0, -dimy/2, 0.05, 0, 0, pi/2);
                    }
                    else if((ay > 0)&&((ax > fabs(ay))&&((yaw > 0 && yaw < pi/2)||(yaw > -pi && yaw < -pi/2)))){
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", -dimx/2, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", -dimx/2-0.05, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", -dimx/2, 0, 0.05, 0, 0, 0);

                        createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", 0, -dimy/2, 0, 0, 0, pi/2);
                        createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", 0, -dimy/2-0.05, 0, 0, 0, pi/2);
                        createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", 0, -dimy/2, 0.05, 0, 0, pi/2);

                        createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", 0, dimy/2, 0, 0, 0, -pi/2);
                        createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", 0, dimy/2+0.05, 0, 0, 0, -pi/2);
                        createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", 0, dimy/2, 0.05, 0, 0, -pi/2);

                        createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, 0, pi/2, 0);
                        createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, 0, pi/2, 0);
                        createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, 0, pi/2, 0);
                    }
                    else if((ay > 0)&&((ax < fabs(ay))&&((yaw > -pi/2 && yaw < 0)||(yaw > pi/2 && yaw < pi)))){
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", dimx/2, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", dimx/2+0.05, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", dimx/2, 0, 0.05, 0, 0, pi);
                    }
                    else if((ay < 0)&&((ax > fabs(ay))&&((yaw > 0 && yaw < pi/2)||(yaw > -pi && yaw < -pi/2)))){
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", 0, dimy/2, 0, 0, 0, -pi/2);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", 0, dimy/2+0.05, 0, 0, 0, -pi/2);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", 0, dimy/2, 0.05, 0, 0, -pi/2);

                        createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", dimx/2, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", dimx/2+0.05, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", dimx/2, 0, 0.05, 0, 0, pi);

                        createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", -dimx/2, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", -dimx/2-0.05, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", -dimx/2, 0, 0.05, 0, 0, 0);

                        createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, 0, pi/2, -pi/2);
                        createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, 0, pi/2, -pi/2);
                        createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, 0, pi/2, -pi/2);
                    }
                    else if((ay < 0)&&((ax < fabs(ay))&&((yaw > -pi/2 && yaw < 0)||(yaw > pi/2 && yaw < pi)))){
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", 0, dimy/2, 0, 0, 0, -pi/2);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", 0, dimy/2+0.05, 0, 0, 0, -pi/2);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", 0, dimy/2, 0.05, 0, 0, -pi/2);
                    }
                    else if((ay < 0)&&(((ax < fabs(ay))&&((yaw > 0 && yaw < pi/2)||(yaw > -pi && yaw < -pi/2))))){
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", dimx/2, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", dimx/2+0.05, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", dimx/2, 0, 0.05, 0, 0, pi);
                    }
                    else{
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", -dimx/2, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", -dimx/2-0.05, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", -dimx/2, 0, 0.05, 0, 0, 0);

                        createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", 0, dimy/2, 0, 0, 0, -pi/2);
                        createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", 0, dimy/2+0.05, 0, 0, 0, -pi/2);
                        createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", 0, dimy/2, 0.05, 0, 0, -pi/2);

                        createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", 0, -dimy/2, 0, 0, 0, pi/2);
                        createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", 0, -dimy/2-0.05, 0, 0, 0, pi/2);
                        createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", 0, -dimy/2, 0.05, 0, 0, pi/2);

                        createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, 0, pi/2, 0);
                        createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, 0, pi/2, 0);
                        createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, 0, pi/2, 0);
                    }
                }
                else if(dimy > gripperlength){
                    if(ay <= 0){
                        if(((yaw < 0 && yaw > -pi/2)&&(ax > fabs(ay)))||((yaw > pi/2 && yaw < pi)&&(ax > fabs(ay)))){
                            createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", 0, -dimy/2, 0, 0, 0, pi/2);
                            createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", 0, -dimy/2-0.05, 0, 0, 0, pi/2);
                            createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", 0, -dimy/2, 0.05, 0, 0, pi/2);

                            createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", 0, dimy/2, 0, 0, 0, -pi/2);
                            createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", 0, dimy/2+0.05, 0, 0, 0, -pi/2);
                            createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", 0, dimy/2, 0.05, 0, 0, -pi/2);

                            createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", 0, 0, dimz/2, 0, pi/2, pi/2);
                            createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, pi/2);
                            createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, pi/2);

                            createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, pi, pi/2, pi/2);
                            createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, pi/2);
                            createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, pi/2);
                        }
                        else{
                            createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", 0, dimy/2, 0, 0, 0, -pi/2);
                            createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", 0, dimy/2+0.05, 0, 0, 0, -pi/2);
                            createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", 0, dimy/2, 0.05, 0, 0, -pi/2);

                            createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", 0, -dimy/2, 0, 0, 0, pi/2);
                            createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", 0, -dimy/2-0.05, 0, 0, 0, pi/2);
                            createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", 0, -dimy/2, 0.05, 0, 0, pi/2);

                            createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", 0, 0, dimz/2, 0, pi/2, -pi/2);
                            createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, -pi/2);
                            createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, -pi/2);

                            createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, pi, pi/2, -pi/2);
                            createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, -pi/2);
                            createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, -pi/2);
                            }
                    }
                    else{
                        if((yaw > -pi/2 && yaw < 0)||(yaw > pi/2 && yaw < pi)||((yaw > 0 && yaw < pi/2)&&(ay > ax))
                        ||((yaw > -pi && yaw < -pi/2)&&(ay > ax))){
                            createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", 0, -dimy/2, 0, 0, 0, pi/2);
                            createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", 0, -dimy/2-0.05, 0, 0, 0, pi/2);
                            createCollisionObjectFrameRPY(bc1, "obj", "obj_lift_pose", 0, -dimy/2, 0.05, 0, 0, pi/2);

                            createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", 0, dimy/2, 0, 0, 0, -pi/2);
                            createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", 0, dimy/2+0.05, 0, 0, 0, -pi/2);
                            createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", 0, dimy/2, 0.05, 0, 0, -pi/2);

                            createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", 0, 0, dimz/2, 0, pi/2, pi/2);
                            createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, pi/2);
                            createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, pi/2);

                            createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, pi, pi/2, pi/2);
                            createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, pi/2);
                            createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, pi/2);
                        }
                        else{
                            createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", 0, dimy/2, 0, 0, 0, -pi/2);
                            createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", 0, dimy/2+0.05, 0, 0, 0, -pi/2);
                            createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", 0, dimy/2, 0.05, 0, 0, -pi/2);

                            createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", 0, -dimy/2, 0, 0, 0, pi/2);
                            createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", 0, -dimy/2-0.05, 0, 0, 0, pi/2);
                            createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", 0, -dimy/2, 0.05, 0, 0, pi/2);

                            createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", 0, 0, dimz/2, 0, pi/2, -pi/2);
                            createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, -pi/2);
                            createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, -pi/2);

                            createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, pi, pi/2, -pi/2);
                            createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, -pi/2);
                            createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, -pi/2);
                            }
                    }
                }
                else if(dimx > gripperlength){
                    if(((ay < 0)&&(fabs(ay) > ax)&&((yaw > 0 && yaw < pi/2)||(yaw > -pi && yaw < -pi/2)))
                    ||((ay > 0)&&(fabs(ay) > ax)&&((yaw > -pi/2 && yaw < 0)||(yaw > pi/2 && yaw < pi)))){
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", dimx/2, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", dimx/2+0.05, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", dimx/2, 0, 0.05, 0, 0, pi);

                        createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", -dimx/2, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", -dimx/2-0.05, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", -dimx/2, 0, 0.05, 0, 0, 0);

                        createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", 0, 0, dimz/2, 0, pi/2, pi);
                        createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, pi);
                        createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, pi);

                        createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, pi, pi/2, pi);
                        createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, pi);
                        createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, pi);
                    }
                    else{
                        createCollisionObjectFrameRPY(bc1, "obj", "obj_grasp_pose", -dimx/2, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc2, "obj", "obj_approach_pose", -dimx/2-0.05, 0, 0, 0, 0, 0);
                        createCollisionObjectFrameRPY(bc3, "obj", "obj_lift_pose", -dimx/2, 0, 0.05, 0, 0, 0);

                        createCollisionObjectFrameRPY(bc4, "obj", "obj_grasp_posea", dimx/2, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc5, "obj", "obj_approach_posea", dimx/2+0.05, 0, 0, 0, 0, pi);
                        createCollisionObjectFrameRPY(bc6, "obj", "obj_lift_posea", dimx/2, 0, 0.05, 0, 0, pi);

                        createCollisionObjectFrameRPY(bc7, "obj", "obj_grasp_poseaa", 0, 0, dimz/2, 0, pi/2, 0);
                        createCollisionObjectFrameRPY(bc8, "obj", "obj_approach_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, 0);
                        createCollisionObjectFrameRPY(bc9, "obj", "obj_lift_poseaa", 0, 0, dimz/2+0.05, 0, pi/2, 0);

                        createCollisionObjectFrameRPY(bc10, "obj", "obj_grasp_poseaaa", 0, 0, dimz/2, pi, pi/2, 0);
                        createCollisionObjectFrameRPY(bc11, "obj", "obj_approach_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, 0);
                        createCollisionObjectFrameRPY(bc12, "obj", "obj_lift_poseaaa", 0, 0, dimz/2+0.05, pi, pi/2, 0);
                    }
                }
            }
            catch (tf::TransformException& ex) {
                ROS_ERROR("%s", ex.what());
            }
            return;
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Broadcast_Grasps_TF");
    ros::NodeHandle nh;
    tffrompose g = tffrompose(&nh);
    ros::spin();
}