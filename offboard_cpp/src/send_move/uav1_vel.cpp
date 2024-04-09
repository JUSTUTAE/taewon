#include "offboardtest.hpp"

class OffboardController 
{
private:
    ros::NodeHandle nh_;
    ros::Publisher set_vel_pub;

    ros::Subscriber state_sub;
    ros::Subscriber ref_path_sub;
    ros::Subscriber current_pose_sub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    mavros_msgs::State current_state;
    nav_msgs::Path ref_path_msg;
    geometry_msgs::PoseStamped current_pose_msg;

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state = *msg;
    }
    void ref_pathCallback(const nav_msgs::Path::ConstPtr& msg) 
    {
        ref_path_msg = *msg;
    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_msg = *msg;
    }
    
public:
    OffboardController() : nh_(""),current_state(),ref_path_msg(),current_pose_msg()
    {
        state_sub = nh_.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, &OffboardController::stateCallback, this);
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
        set_mode_client= nh_.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");
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


                // Wait for FCU connection
        while (ros::ok() && !current_state.connected) 
        {
            ros::spinOnce();
            rate.sleep();
        } 
          //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i)
        {
        set_vel_pub.publish(setvel);
        ros::spinOnce();
        rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();

        while (ros::ok()) 
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) 
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
                {
                    ROS_INFO("uav1 Offboard enabled");
                }
                last_request = ros::Time::now();
            } 
            else 
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) 
                    {
                        ROS_INFO("uav1 armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            z = ref_path_msg.poses[0].pose.position.z-current_pose_msg.pose.position.z;
            x = ref_path_msg.poses[0].pose.position.x-current_pose_msg.pose.position.x;
            y = ref_path_msg.poses[0].pose.position.y-current_pose_msg.pose.position.y-5;

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