#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath> 
#include <nav_msgs/Path.h> //path
#include <geometry_msgs/PointStamped.h> //point

class OffboardController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Publisher local_pos_pub_;
    ros::Publisher path_pub_; //path
    ros::Publisher point_pub_; //point
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::Subscriber pose_sub_;

    ros::Subscriber pose1_sub_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped current_pose1_;
    nav_msgs::Path path; //path
    geometry_msgs::PointStamped point; //point

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }

    void pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose1_ = *msg;
    }

public:
    OffboardController() : nh_(""), current_pose_(), current_state_(),current_pose1_() {
        state_sub_ = nh_.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, &OffboardController::stateCallback, this);
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &OffboardController::poseCallback, this);
        
        pose1_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, &OffboardController::pose1Callback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>("uav1/path", 10); //path
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("uav1/point", 10); //point
    }

    void run() {
        ros::Rate rate(20.0);

        // Wait for FCU connection
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = -2.5;
        pose.pose.position.y = -2;
        pose.pose.position.z = 2;

        // Send a few setpoints before starting
        for (int i = 100; ros::ok() && i > 0; --i) {
            local_pos_pub_.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();


        while (ros::ok()) {
            if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("uav1 Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("uav1 armed");
                    }
                    last_request = ros::Time::now();
                }
                pose.pose.position.x = (current_pose_.pose.position.x)-2.5;
                pose.pose.position.y = (current_pose_.pose.position.y)-2;
                pose.pose.position.z = 2;

                //
                point.point.x=(current_pose_.pose.position.x)-2.5;
                point.point.y=(current_pose_.pose.position.y)-2;
                point.point.z=2;//point
            }

            local_pos_pub_.publish(pose);

             //
            path.header.frame_id="map";
            path.header.stamp=ros::Time::now();
            path.poses.emplace_back(current_pose1_);
            path_pub_.publish(path); 
            //path

            //
            point.header.frame_id="map";
            point.header.stamp=ros::Time::now();
            point_pub_.publish(point); 
            //point


            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav1_offb_node");

    OffboardController offboard_controller;
    offboard_controller.run();

    return 0;
}


