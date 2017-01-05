#include<ros/ros.h>
#include<std_msgs/String.h>
#include<nav_april_laser_odom/command.h>
int move[7]={3 ,2,3,2,3,2,3};
/*
double plan[7][3]={1.2,0,0,
                               1.2,0,-M_PI/2,
                               1.2,-1.2,-M_PI/2,
                               1.2,-1.2,-M_PI,
                               0,-1.2,-M_PI,
                               0,-1.2,-M_PI/2,
                               0,-1.8,-M_PI/2};
                               */
double plan[7][3]={1.2,0,0,
                               1.2,0,-M_PI/2,
                               1.2,-1.2,-M_PI/2,
                               1.2,-1.2,-M_PI,
                               0,-1.2,-M_PI,
                               0,-1.2,M_PI/2,
                               0,0,M_PI/2};
int i=0;
ros::Publisher command;
//std_msgs::String path;
nav_april_laser_odom::command path;
void reachCB(const std_msgs::String msg)
{
    if(i<7)
    {
        ROS_INFO("GET THE PATH");
        //path.data=plans[i];
        path.command=move[i];
        path.pose.x=plan[i][0];
        path.pose.y=plan[i][1];
        path.pose.theta=plan[i][2];
        command.publish(path);
        i++;
    }
    else
    {
        ROS_INFO("I AM IN THE END OF PLAN!");
    }
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"command");
    ros::NodeHandle nh;
    ros::Subscriber reachflag=nh.subscribe("/reachflag",1,reachCB);
    command=nh.advertise<nav_april_laser_odom::command>("command",1);

    ros::spin();

}


