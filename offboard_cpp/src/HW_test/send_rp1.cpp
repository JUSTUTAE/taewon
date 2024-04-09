#include "offboardtest.hpp"

class Sendtrajectory 
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_1_sub;

    ros::Publisher r_path_1_pub;
  //ref_path_publish

    ros::Publisher path_1_pub; 
 //current_path_publish

    geometry_msgs::PoseStamped current_pose_1;

    nav_msgs::Path uav1_path; 


    void pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_1 = *msg;
    }

public:
    Sendtrajectory() : nh_(""), current_pose_1()
    {
        pose_1_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, &Sendtrajectory::pose1Callback, this);

        r_path_1_pub= nh_.advertise<nav_msgs::Path>("uav1/ref_path", 10);
      
        path_1_pub = nh_.advertise<nav_msgs::Path>("uav1/path", 10); 
    }

    void run() 
    {
        ros::Rate rate(10.0);

        nav_msgs::Path ref_path_1_msg;

        geometry_msgs::PoseStamped refpoint1;

        double temp_x,temp_y,x_r,y_r=0; 

        ref_path_1_msg.header.frame_id = "map";

        uav1_path.header.frame_id="map";


        while (ros::ok()) 
        {
            if (current_pose_1.pose.position.z >= 1.8) 
            {
                for(int i = 0; i < 10; ++i)
                {  
                    if(x_r<36) //3min
                    {
                        x_r = temp_x + 0.02 * i;
                        y_r = 3 * sin(2 * M_PI / 5 * x_r);
                    }
                    else
                    {
                        x_r=x_r;
                        y_r=y_r;
                    }

                    if(i == 1)
                    {   
                        temp_x = x_r;
                        temp_y = y_r;
                    }
                    refpoint1.pose.position.x=x_r;
                    refpoint1.pose.position.y=y_r;
                    refpoint1.pose.position.z=2.0;
                    ref_path_1_msg.header.stamp = ros::Time::now();
                    ref_path_1_msg.poses.emplace_back(refpoint1);
                }
            }
            else
            {
                refpoint1.pose.position.x=x_r;
                refpoint1.pose.position.y=y_r;
                refpoint1.pose.position.z=2.0;

                ref_path_1_msg.header.stamp = ros::Time::now();
                ref_path_1_msg.poses.emplace_back(refpoint1);
            } 
             //path_publish
            uav1_path.header.stamp=ros::Time::now();
            uav1_path.poses.emplace_back(current_pose_1);
            path_1_pub.publish(uav1_path);  

            //ref_path_publish
            r_path_1_pub.publish(ref_path_1_msg);
            ref_path_1_msg.poses.clear();        

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "send_traj_node1");

    Sendtrajectory send_traj;
    send_traj.run();

    return 0;
}

