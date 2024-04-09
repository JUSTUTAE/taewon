#include "offboardtest.hpp"

class errorpub
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber vel_0_sub;
    ros::Subscriber vel_1_sub;
    ros::Subscriber vel_2_sub;
    ros::Subscriber vel_3_sub;

    ros::Subscriber set_vel_0_sub;
    ros::Subscriber set_vel_1_sub;
    ros::Subscriber set_vel_2_sub;
    ros::Subscriber set_vel_3_sub;

    ros::Publisher error_0_pub;
    ros::Publisher error_1_pub;
    ros::Publisher error_2_pub;
    ros::Publisher error_3_pub;

    geometry_msgs::TwistStamped current_v_0;
    geometry_msgs::TwistStamped current_v_1;
    geometry_msgs::TwistStamped current_v_2;
    geometry_msgs::TwistStamped current_v_3;

    geometry_msgs::TwistStamped set_v_0;
    geometry_msgs::TwistStamped set_v_1;
    geometry_msgs::TwistStamped set_v_2;
    geometry_msgs::TwistStamped set_v_3;

    void vel0Callback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
    {
        current_v_0 = *msg;
    }
    void vel1Callback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
    {
        current_v_1= *msg;
    }
    void vel2Callback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
    {
        current_v_2 = *msg;
    }
    void vel3Callback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
    {
        current_v_3 = *msg;
    }

        void setv0Callback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
    {
        set_v_0 = *msg;
    }
    void setv1Callback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
    {
        set_v_1= *msg;
    }
    void setv2Callback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
    {
        set_v_2 = *msg;
    }
    void setv3Callback(const geometry_msgs::TwistStamped::ConstPtr& msg) 
    {
        set_v_3 = *msg;
    }

public:
    errorpub() : nh_(""), current_v_0(),current_v_1(),current_v_2(),current_v_3(),set_v_0(),set_v_1(),set_v_2(),set_v_3()
    {
        vel_0_sub = nh_.subscribe<geometry_msgs::TwistStamped>("uav0/mavros/local_position/velocity_local", 10, &errorpub::vel0Callback, this);
        vel_1_sub = nh_.subscribe<geometry_msgs::TwistStamped>("uav1/mavros/local_position/velocity_local", 10, &errorpub::vel1Callback, this);
        vel_2_sub = nh_.subscribe<geometry_msgs::TwistStamped>("uav2/mavros/local_position/velocity_local", 10, &errorpub::vel2Callback, this);
        vel_3_sub = nh_.subscribe<geometry_msgs::TwistStamped>("uav3/mavros/local_position/velocity_local", 10, &errorpub::vel3Callback, this);

        set_vel_0_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/uav0/mavros/setpoint_velocity/cmd_vel", 10, &errorpub::setv0Callback, this);
        set_vel_1_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/setpoint_velocity/cmd_vel", 10, &errorpub::setv1Callback, this);
        set_vel_2_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/setpoint_velocity/cmd_vel", 10, &errorpub::setv2Callback, this); 
        set_vel_3_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/uav3/mavros/setpoint_velocity/cmd_vel", 10, &errorpub::setv3Callback, this);

        error_0_pub = nh_.advertise<geometry_msgs::TwistStamped>("uav0/error", 10);
        error_1_pub = nh_.advertise<geometry_msgs::TwistStamped>("uav1/error", 10);
        error_2_pub = nh_.advertise<geometry_msgs::TwistStamped>("uav2/error", 10);
        error_3_pub = nh_.advertise<geometry_msgs::TwistStamped>("uav3/error", 10);
    }

    void run() 
    {
        ros::Rate rate(10.0);
        geometry_msgs::TwistStamped uav0;
        geometry_msgs::TwistStamped uav1;
        geometry_msgs::TwistStamped uav2;
        geometry_msgs::TwistStamped uav3;

        while (ros::ok()) 
        {
            uav0.twist.linear.x=set_v_0.twist.linear.x-current_v_0.twist.linear.x;
            uav0.twist.linear.y=set_v_0.twist.linear.y-current_v_0.twist.linear.y;
            uav0.twist.linear.z=set_v_0.twist.linear.z-current_v_0.twist.linear.z;

            uav1.twist.linear.x=set_v_1.twist.linear.x-current_v_1.twist.linear.x;
            uav1.twist.linear.y=set_v_1.twist.linear.y-current_v_1.twist.linear.y;
            uav1.twist.linear.z=set_v_1.twist.linear.z-current_v_1.twist.linear.z;

            uav2.twist.linear.x=set_v_2.twist.linear.x-current_v_2.twist.linear.x;
            uav2.twist.linear.y=set_v_2.twist.linear.y-current_v_2.twist.linear.y;
            uav2.twist.linear.z=set_v_2.twist.linear.z-current_v_2.twist.linear.z;

            uav3.twist.linear.x=set_v_3.twist.linear.x-current_v_3.twist.linear.x;
            uav3.twist.linear.y=set_v_3.twist.linear.y-current_v_3.twist.linear.y;
            uav3.twist.linear.z=set_v_3.twist.linear.z-current_v_3.twist.linear.z;

            error_0_pub.publish(uav0);
            error_1_pub.publish(uav1);
            error_2_pub.publish(uav2);
            error_3_pub.publish(uav3);

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "errorpub");

    errorpub errorpub;
    errorpub.run();

    return 0;
}

