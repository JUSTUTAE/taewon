#include "offboardtest.hpp"

class Sendtrajectory 
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_0_sub;
    ros::Subscriber pose_1_sub;
    ros::Subscriber pose_2_sub;
    ros::Subscriber pose_3_sub;
    ros::Subscriber formation_sub;

    ros::Publisher r_path_0_pub;
    ros::Publisher r_path_1_pub; 
    ros::Publisher r_path_2_pub;
    ros::Publisher r_path_3_pub;  //ref_path_publish

    ros::Publisher path_0_pub; 
    ros::Publisher path_1_pub; 
    ros::Publisher path_2_pub; 
    ros::Publisher path_3_pub;  //current_path_publish

    ros::Publisher gpose_1_pub; 
    ros::Publisher gpose_2_pub;
    ros::Publisher gpose_3_pub;  //uav1,2 global_pose_publish

    geometry_msgs::PoseStamped current_pose_0;
    geometry_msgs::PoseStamped current_pose_1;
    geometry_msgs::PoseStamped current_pose_2;
    geometry_msgs::PoseStamped current_pose_3;

    std_msgs::Int64 f_num;

    nav_msgs::Path uav0_path; 
    nav_msgs::Path uav1_path; 
    nav_msgs::Path uav2_path; 
    nav_msgs::Path uav3_path; 

    void pose0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_0 = *msg;
    }
    void pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_1= *msg;
    }
    void pose2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_2 = *msg;
    }
    void pose3Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_3 = *msg;
    }
    void formCallback(const std_msgs::Int64::ConstPtr& msg) 
    {
        f_num = *msg;
    }

public:
    Sendtrajectory() : nh_(""), current_pose_0(),current_pose_1(),current_pose_2(),current_pose_3(),f_num()
    {
        pose_0_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &Sendtrajectory::pose0Callback, this);
        pose_1_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, &Sendtrajectory::pose1Callback, this);
        pose_2_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav2/mavros/local_position/pose", 10, &Sendtrajectory::pose2Callback, this);
        pose_3_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav3/mavros/local_position/pose", 10, &Sendtrajectory::pose3Callback, this);
        formation_sub = nh_.subscribe<std_msgs::Int64>("/formation_num", 10, &Sendtrajectory::formCallback, this);

        r_path_0_pub= nh_.advertise<nav_msgs::Path>("uav0/ref_path", 10);
        r_path_1_pub= nh_.advertise<nav_msgs::Path>("uav1/ref_path", 10);
        r_path_2_pub= nh_.advertise<nav_msgs::Path>("uav2/ref_path", 10);
        r_path_3_pub= nh_.advertise<nav_msgs::Path>("uav3/ref_path", 10);

        path_0_pub = nh_.advertise<nav_msgs::Path>("uav0/path", 10); 
        path_1_pub = nh_.advertise<nav_msgs::Path>("uav1/path", 10); 
        path_2_pub = nh_.advertise<nav_msgs::Path>("uav2/path", 10); 
        path_3_pub = nh_.advertise<nav_msgs::Path>("uav3/path", 10); 

        gpose_1_pub = nh_.advertise<geometry_msgs::PoseStamped>("uav1/global_current_pose", 10);
        gpose_2_pub = nh_.advertise<geometry_msgs::PoseStamped>("uav2/global_current_pose", 10);
        gpose_3_pub = nh_.advertise<geometry_msgs::PoseStamped>("uav3/global_current_pose", 10);
    }

    void run() 
    {
        ros::Rate rate(10.0);

        nav_msgs::Path ref_path_0_msg;
        nav_msgs::Path ref_path_1_msg;
        nav_msgs::Path ref_path_2_msg;
        nav_msgs::Path ref_path_3_msg;

        geometry_msgs::PoseStamped refpoint0;
        geometry_msgs::PoseStamped refpoint1;
        geometry_msgs::PoseStamped refpoint2;
        geometry_msgs::PoseStamped refpoint3;

        geometry_msgs::PoseStamped gpose1;
        geometry_msgs::PoseStamped gpose2;
        geometry_msgs::PoseStamped gpose3; 

        double temp_x,temp_y,x_r,y_r=0; 

        ref_path_0_msg.header.frame_id = "map";
        ref_path_1_msg.header.frame_id = "map";
        ref_path_2_msg.header.frame_id = "map";
        ref_path_3_msg.header.frame_id = "map";

        uav0_path.header.frame_id="map";
        uav1_path.header.frame_id="map";
        uav2_path.header.frame_id="map";
        uav3_path.header.frame_id="map";

        gpose1.header.frame_id="map";
        gpose2.header.frame_id="map";
        gpose3.header.frame_id="map";

        gpose1.pose.position.x=current_pose_1.pose.position.x;
        gpose1.pose.position.y=current_pose_1.pose.position.y+5;
        gpose1.pose.position.z=current_pose_1.pose.position.z;

        gpose2.pose.position.x=current_pose_2.pose.position.x;
        gpose2.pose.position.y=current_pose_2.pose.position.y-5;
        gpose2.pose.position.z=current_pose_2.pose.position.z;

        gpose3.pose.position.x=current_pose_3.pose.position.x-5;
        gpose3.pose.position.y=current_pose_3.pose.position.y;
        gpose3.pose.position.z=current_pose_3.pose.position.z;

        double fm[3][4][3]= {   {{0,0,0},{-2.5,-2,0},{-2.5,2,0},{2.5,0,0}},
                                {{0,0,2},{0,-2,2},{0,-1,2},{5,-3,2}},
                                {{3,2,1},{0,-3,1},{3,3,1},{5,-2,1}}
                            };

        while (ros::ok()) 
        {
            if (current_pose_0.pose.position.z >= 1.8) 
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
                    if(f_num.data==1)
                    {
                        refpoint0.pose.position.x=x_r+fm[0][0][0];
                        refpoint0.pose.position.y=y_r+fm[0][0][1];
                        refpoint0.pose.position.z=2.0+fm[0][0][2];

                        refpoint1.pose.position.x=x_r+fm[0][1][0];
                        refpoint1.pose.position.y=y_r+5+fm[0][1][1];
                        refpoint1.pose.position.z=2.0+fm[0][1][2];

                        refpoint2.pose.position.x=x_r+fm[0][2][0];
                        refpoint2.pose.position.y=y_r-5+fm[0][2][1];
                        refpoint2.pose.position.z=2.0+fm[0][2][2];

                        refpoint3.pose.position.x=x_r-5+fm[0][3][0];
                        refpoint3.pose.position.y=y_r+fm[0][3][1];
                        refpoint3.pose.position.z=2.0+fm[0][3][2];

                        ref_path_0_msg.header.stamp = ros::Time::now();
                        ref_path_0_msg.poses.emplace_back(refpoint0);

                        ref_path_1_msg.header.stamp = ros::Time::now();
                        ref_path_1_msg.poses.emplace_back(refpoint1);

                        ref_path_2_msg.header.stamp = ros::Time::now();
                        ref_path_2_msg.poses.emplace_back(refpoint2);

                        ref_path_3_msg.header.stamp = ros::Time::now();
                        ref_path_3_msg.poses.emplace_back(refpoint3);
                    }
                    else if(f_num.data==2)
                    {
                        refpoint0.pose.position.x=x_r+fm[1][0][0];
                        refpoint0.pose.position.y=y_r+fm[1][0][1];
                        refpoint0.pose.position.z=2.0+fm[1][0][2];

                        refpoint1.pose.position.x=x_r+fm[1][1][0];
                        refpoint1.pose.position.y=y_r+5+fm[1][1][1];
                        refpoint1.pose.position.z=2.0+fm[1][1][2];

                        refpoint2.pose.position.x=x_r+fm[1][2][0];
                        refpoint2.pose.position.y=y_r-5+fm[1][2][1];
                        refpoint2.pose.position.z=2.0+fm[1][2][2];

                        refpoint3.pose.position.x=x_r-5+fm[1][3][0];
                        refpoint3.pose.position.y=y_r+fm[1][3][1];
                        refpoint3.pose.position.z=2.0+fm[1][3][2];

                        ref_path_0_msg.header.stamp = ros::Time::now();
                        ref_path_0_msg.poses.emplace_back(refpoint0);

                        ref_path_1_msg.header.stamp = ros::Time::now();
                        ref_path_1_msg.poses.emplace_back(refpoint1);

                        ref_path_2_msg.header.stamp = ros::Time::now();
                        ref_path_2_msg.poses.emplace_back(refpoint2);

                        ref_path_3_msg.header.stamp = ros::Time::now();
                        ref_path_3_msg.poses.emplace_back(refpoint3);
                    }
                    else if(f_num.data==3)
                    {
                        refpoint0.pose.position.x=x_r+fm[2][0][0];
                        refpoint0.pose.position.y=y_r+fm[2][0][1];
                        refpoint0.pose.position.z=2.0+fm[2][0][2];

                        refpoint1.pose.position.x=x_r+fm[2][1][0];
                        refpoint1.pose.position.y=y_r+5+fm[2][1][1];
                        refpoint1.pose.position.z=2.0+fm[2][1][2];

                        refpoint2.pose.position.x=x_r+fm[2][2][0];
                        refpoint2.pose.position.y=y_r-5+fm[2][2][1];
                        refpoint2.pose.position.z=2.0+fm[2][2][2];

                        refpoint3.pose.position.x=x_r-5+fm[2][3][0];
                        refpoint3.pose.position.y=y_r+fm[2][3][1];
                        refpoint3.pose.position.z=2.0+fm[2][3][2];

                        ref_path_0_msg.header.stamp = ros::Time::now();
                        ref_path_0_msg.poses.emplace_back(refpoint0);

                        ref_path_1_msg.header.stamp = ros::Time::now();
                        ref_path_1_msg.poses.emplace_back(refpoint1);

                        ref_path_2_msg.header.stamp = ros::Time::now();
                        ref_path_2_msg.poses.emplace_back(refpoint2);

                        ref_path_3_msg.header.stamp = ros::Time::now();
                        ref_path_3_msg.poses.emplace_back(refpoint3);
                    }
                }
            }
            else
            {
                refpoint0.pose.position.x=x_r;
                refpoint0.pose.position.y=y_r;
                refpoint0.pose.position.z=2.0;

                refpoint1.pose.position.x=x_r-2.5;
                refpoint1.pose.position.y=y_r-2+5;
                refpoint1.pose.position.z=2.0;

                refpoint2.pose.position.x=x_r-2.5;
                refpoint2.pose.position.y=y_r+2-5;
                refpoint2.pose.position.z=2.0;

                refpoint3.pose.position.x=x_r+2.5-5;
                refpoint3.pose.position.y=y_r;
                refpoint3.pose.position.z=2.0;

                ref_path_0_msg.header.stamp = ros::Time::now();
                ref_path_0_msg.poses.emplace_back(refpoint0);

                ref_path_1_msg.header.stamp = ros::Time::now();
                ref_path_1_msg.poses.emplace_back(refpoint1);

                ref_path_2_msg.header.stamp = ros::Time::now();
                ref_path_2_msg.poses.emplace_back(refpoint2);

                ref_path_3_msg.header.stamp = ros::Time::now();
                ref_path_3_msg.poses.emplace_back(refpoint3);
            } 
            gpose1.pose.position.x=current_pose_1.pose.position.x;
            gpose1.pose.position.y=current_pose_1.pose.position.y+5;
            gpose1.pose.position.z=current_pose_1.pose.position.z;

            gpose2.pose.position.x=current_pose_2.pose.position.x;
            gpose2.pose.position.y=current_pose_2.pose.position.y-5;
            gpose2.pose.position.z=current_pose_2.pose.position.z;

            gpose3.pose.position.x=current_pose_3.pose.position.x-5;
            gpose3.pose.position.y=current_pose_3.pose.position.y;
            gpose3.pose.position.z=current_pose_3.pose.position.z;

            //global_pose_publish       

            gpose1.header.stamp=ros::Time::now();
            gpose_1_pub.publish(gpose1);
            gpose2.header.stamp=ros::Time::now();
            gpose_2_pub.publish(gpose2);
            gpose3.header.stamp=ros::Time::now();
            gpose_3_pub.publish(gpose3);

             //path_publish
            uav0_path.header.stamp=ros::Time::now();
            uav0_path.poses.emplace_back(current_pose_0);
            path_0_pub.publish(uav0_path); 

            uav1_path.header.stamp=ros::Time::now();
            uav1_path.poses.emplace_back(gpose1);
            path_1_pub.publish(uav1_path); 

            uav2_path.header.stamp=ros::Time::now();
            uav2_path.poses.emplace_back(gpose2);
            path_2_pub.publish(uav2_path);    

            uav3_path.header.stamp=ros::Time::now();
            uav3_path.poses.emplace_back(gpose3);
            path_3_pub.publish(uav3_path);      

            //ref_path_publish
            r_path_0_pub.publish(ref_path_0_msg);
            ref_path_0_msg.poses.clear();

            r_path_1_pub.publish(ref_path_1_msg);
            ref_path_1_msg.poses.clear();

            r_path_2_pub.publish(ref_path_2_msg);
            ref_path_2_msg.poses.clear();      

            r_path_3_pub.publish(ref_path_3_msg);
            ref_path_3_msg.poses.clear();              

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "send_traj_node");

    Sendtrajectory send_traj;
    send_traj.run();

    return 0;
}

