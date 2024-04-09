#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h> //path
#include <geometry_msgs/PointStamped.h> //p
#include <cmath>

class OffboardController {
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Publisher local_pos_pub_;
    ros::Publisher path_pub_; //path
    ros::Publisher point_pub_; //p
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::Subscriber pose_sub_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    nav_msgs::Path path; //path
    geometry_msgs::PointStamped point; //p
  

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }


public:
    OffboardController() : nh_(""), current_pose_(), current_state_() {
        state_sub_ = nh_.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, &OffboardController::stateCallback, this);
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode");
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &OffboardController::poseCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("uav0/path", 10); //path
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("uav0/point", 10); //p
    }

    void run() {
        ros::Rate rate(20.0);

        // Wait for FCU connection
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
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

        float x_r = 0;
        float y_r = 0;
        //float d_x=0;
        //float d_y=0;
        //float D=0;
        while (ros::ok()) {
            if (current_pose_.pose.position.z >= 2) {
               // d_x=pose.pose.position.x-current_pose_.pose.position.x;
               // d_y=pose.pose.position.y-current_pose_.pose.position.y;
               // D=(d_x)*(d_x)+(d_y)*(d_y);
                //if(sqrt(D)>=0 && sqrt(D)<=0.5)
                //{
                    if (x_r < 15) {
                    x_r += 0.02;
                }
                    y_r = 3 * sin(2 * M_PI / 5 * x_r);
               // }
            }

            if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("uav0 Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("uav0 armed");
                    }
                    last_request = ros::Time::now();
                }
                pose.pose.position.x = x_r;
                pose.pose.position.y = y_r;
                pose.pose.position.z = 2;
                
                point.point.x=x_r;
                point.point.y=y_r;
                point.point.z=2;
            }

            local_pos_pub_.publish(pose);
            
            //
            path.header.frame_id="map";
            path.header.stamp=ros::Time::now();
            path.poses.emplace_back(current_pose_);
            path_pub_.publish(path); 
            //path

            point.header.frame_id="map";
            point.header.stamp=ros::Time::now();
            point_pub_.publish(point); 

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav0_offb_node");

    OffboardController offboard_controller;
    offboard_controller.run();

    return 0;
}

