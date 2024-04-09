#include "offboardtest.hpp"

class OffboardController 
{
private:
    ros::NodeHandle nh_;
    ros::Publisher set_vel_pub;

    ros::Subscriber ref_path_sub;
    ros::Subscriber current_pose_sub;

    nav_msgs::Path ref_path_msg;
    geometry_msgs::PoseStamped current_pose_msg;

    void ref_pathCallback(const nav_msgs::Path::ConstPtr& msg) 
    {
        ref_path_msg = *msg;
    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_msg = *msg;
    }
    
public:
    OffboardController() : nh_(""),ref_path_msg(),current_pose_msg()
    {
        ref_path_sub = nh_.subscribe<nav_msgs::Path>("uav1/ref_path", 10, &OffboardController::ref_pathCallback, this);
        current_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, &OffboardController::poseCallback, this);

        set_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/uav1/mavros/setpoint_velocity/cmd_vel", 10);
    }

    void run() 
    {
        ros::Rate rate(10.0);

        double x,y,z=0;

        geometry_msgs::TwistStamped setvel;

        setvel.twist.linear.x=0;
        setvel.twist.linear.y=0;
        setvel.twist.linear.z=0.2;

          //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i)
        {
        set_vel_pub.publish(setvel);
        ros::spinOnce();
        rate.sleep();
        }

        while (ros::ok()) 
        {
            z = ref_path_msg.poses[0].pose.position.z-current_pose_msg.pose.position.z;
            x = ref_path_msg.poses[0].pose.position.x-current_pose_msg.pose.position.x;
            y = ref_path_msg.poses[0].pose.position.y-current_pose_msg.pose.position.y;

            setvel.twist.linear.x=x;
            setvel.twist.linear.y=y;
            setvel.twist.linear.z=z;
            
            set_vel_pub.publish(setvel);

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "uav1_move_node");

    OffboardController offboard_controller;
    offboard_controller.run();

    return 0;
}