#include "offboardtest.hpp"

class Send_formation
{
private:
    ros::NodeHandle nh_;
    ros::Publisher formation_num_pub;

    ros::Subscriber pose_0_sub;

    geometry_msgs::PoseStamped current_pose_0;

    void pose0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_0 = *msg;
    }
    
public:
    Send_formation() : nh_(""),current_pose_0()
    {
        pose_0_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &Send_formation::pose0Callback, this);
        formation_num_pub = nh_.advertise<std_msgs::Int64>("/formation_num", 10);
    }

    void run() 
    {
        ros::Rate rate(10.0);

        int num=0,i=0;
        std_msgs::Int64 f_num;

        while (ros::ok())
        {
            if(current_pose_0.pose.position.z>=1.8) 
            {
                if(i<600 && i>=0) //rate=10s , 1s=>1=10, i600=1m
                {
                    i=i+1;
                    num=1;
                }
                else if(i>=600&&i<1200)
                {
                    i=i+1;
                    num=2;
                }
                else
                {
                    i=i;
                    num=3;
                }
            }
            f_num.data=num;

            formation_num_pub.publish(f_num);
        
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "send_formation_node");

    Send_formation send_formation;
    send_formation.run();

    return 0;
}