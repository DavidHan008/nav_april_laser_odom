#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<cmath>
#include<algorithm>
#include<ros/timer.h>
#include<nav_msgs/Odometry.h>
#include<std_msgs/String.h>
double goal_x=0,goal_theta=0;
double current_x,current_y,current_theta;
ros::Time t_last;
double last_x=0,last_y=0,last_theta=0;
double v_x,v_y,v_theta;
double abs_x,abs_y,abs_theta;
//bool flag_x,flag_y,flag_theta;
double P_turn=1.0;
double P_straight=1.1;
double v_xmax=0.2;
double v_turnmax=0.6;
double xy_tolerance=0.005,theta_tolerance=0.01;
ros::Subscriber odom_sub;
ros::Subscriber command;
std::string c1="1",c2="2",c3="3",c4="4";
int flag=4;
ros::Publisher  control_pub;
ros::Publisher reachflag;
geometry_msgs::Twist cmdvel;
std_msgs::String reachOK;
std::string ok="OK";
bool firsttime=true;
void clearTwist()
{
    cmdvel.angular.x=0;
    cmdvel.angular.y=0;
    cmdvel.angular.z=0;
    cmdvel.linear.x=0;
    cmdvel.linear.y=0;
    cmdvel.linear.z=0;
}
bool turnLeft()
{
        ROS_INFO_STREAM(goal_theta);
        if(goal_theta==M_PI&&current_theta<0)
        {
            goal_theta=-M_PI;
        }
        else if(goal_theta==-M_PI&&current_theta>0)
        {
            goal_theta=M_PI;
        }
    double dtheta=goal_theta-current_theta;
    ROS_INFO("dtheta is%f",dtheta);
    bool flag_theta;
    clearTwist();
    if(fabs(dtheta)>=theta_tolerance)
    {
        cmdvel.angular.z=std::max(std::min(P_turn*dtheta,v_turnmax),-v_turnmax);
        control_pub.publish(cmdvel);
        flag_theta=false;
    }
    else
    {
        ROS_INFO("i am done!");
        clearTwist();
        cmdvel.angular.z=-cmdvel.angular.z*0.35;
        control_pub.publish(cmdvel);

        flag=4;
        flag_theta=true;
    }
    return flag_theta;
}
bool turnRight()
{
        if(goal_theta==M_PI&&current_theta<0)
        {
            goal_theta=-M_PI;
        }
        else if(goal_theta==-M_PI&&current_theta>0)
        {
            goal_theta=M_PI;
        }
    double dtheta=goal_theta-current_theta;
    //ROS_INFO("dtheta is%f",dtheta);
    bool flag_theta;
    clearTwist();
    if(fabs(dtheta)>=theta_tolerance)
    {
        cmdvel.angular.z=std::max(std::min(P_turn*dtheta,v_turnmax),-v_turnmax);
        control_pub.publish(cmdvel);
        flag_theta=false;
    }
    else
    {
        ROS_INFO("i am done!");
        clearTwist();
        control_pub.publish(cmdvel);
        cmdvel.angular.z=-cmdvel.angular.z*0.35;
        flag=4;
        flag_theta=true;
    }
    return flag_theta;
}
bool goStraight()
{
    double dx=goal_x-current_x;
    bool flag_x;
    clearTwist();
    if(fabs(dx)>=xy_tolerance)
    {
        cmdvel.linear.x=std::max(std::min(P_straight*dx,v_xmax),-v_xmax);
        control_pub.publish(cmdvel);
        flag_x=false;
    }
    else
    {
        ROS_INFO("i am done!");
        ROS_INFO_STREAM(current_x);
        clearTwist();
        control_pub.publish(cmdvel);
        flag=4;
        flag_x=true;
    }
    return flag_x;
}
void odomCB(const nav_msgs::Odometry::ConstPtr msg)
{
    //ROS_INFO_STREAM("hahahha");
    ros::Time t_now=ros::Time::now();
    tf::Quaternion q;
    q.setW(msg->pose.pose.orientation.w);
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    current_theta=q.getAngle() * q.getAxis().getZ();
    if(current_theta>M_PI)
    {
        current_theta-=2*M_PI;
    }
    else if(current_theta<-M_PI)
    {
        current_theta+=2*M_PI;
    }
    if(-0.1745<current_theta&&current_theta<0.1745)//0 degree
    {
        current_x=msg->pose.pose.position.x;
        current_y=msg->pose.pose.position.y;
    }
    else if(1.3963<current_theta&&current_theta<1.7453)//90 degree
    {
        current_x=msg->pose.pose.position.y;
        current_y=-msg->pose.pose.position.x;
    }
    else if(2.967<current_theta||current_theta<-2.967)//180 degree
    {
        current_x=-msg->pose.pose.position.x;
        current_y=-msg->pose.pose.position.y;
    }
    else if(-1.7453<current_theta&&current_theta<-1.3963)//-90 degree
    {
        current_x=-msg->pose.pose.position.y;
        current_y=msg->pose.pose.position.x;
    }
ROS_INFO("current_theta:%f",current_theta*180/3.1415926);

   ros::Duration dt=t_now-t_last;
    v_x=(current_x-last_x)/dt.toSec();
    v_y=(current_y-last_y)/dt.toSec();
    v_theta=(current_theta-last_theta)/dt.toSec();
    ROS_INFO("v_x:%f,v_y:%f,v_theta:%f",v_x,v_y,v_theta);
        last_x=current_x;
        last_y=current_y;
        last_theta=current_theta;
        t_last=t_now;

}
void timeCB(const ros::TimerEvent&)
{
    if(firsttime==true)
    {
        //firsttime=false;
        reachOK.data=ok;
        reachflag.publish(reachOK);
    }
    if(flag==1) turnLeft();
    else if(flag==2) turnRight();
    else if(flag==3) goStraight();
    else if(flag==4)
    {
        if(fabs(v_x)<0.001&&fabs(v_y)<0.001&&fabs(v_theta)<0.0005)
        {
            reachOK.data=ok;
            reachflag.publish(reachOK);
        }
        clearTwist();
        control_pub.publish(cmdvel);
    }
    else
    {
        clearTwist();
        control_pub.publish(cmdvel);
    }
}
void commandCB(const std_msgs::String::ConstPtr msg)
{
    firsttime=false;
    int flagg=atoi((const char * )(msg->data.data()));
    abs_x=current_x;
    abs_y=current_y;
    abs_theta=current_theta;
    ROS_INFO_STREAM(flagg);
    ROS_INFO("abs_x:%f,abs_y:%f,abs_theta:%f",abs_x,abs_y,abs_theta*180/3.1415926);
    if(flagg==1)//turn left
    {
        flag=1;
        goal_theta=abs_theta+M_PI/2;
        if(fabs(goal_theta)<0.1745) goal_theta=0;
        else if(fabs(goal_theta-M_PI/2)<0.1745) goal_theta=M_PI/2;
        else if(fabs(goal_theta+M_PI/2)<0.1745) goal_theta=-M_PI/2;
        //else if(fabs(goal_theta+M_PI)<0.1745) goal_theta=M_PI;
        else if(fabs(goal_theta-M_PI)<0.1745&&goal_theta>0) goal_theta=M_PI;
        else if(fabs(goal_theta+M_PI)<0.1745&&goal_theta<0) goal_theta=-M_PI;

    }
    else if(flagg==2)//turn right
    {
        flag=2;
        goal_theta=abs_theta-M_PI/2;
        if(fabs(goal_theta)<0.1745) goal_theta=0;
        else if(fabs(goal_theta-M_PI/2)<0.1745) goal_theta=M_PI/2;
        else if(fabs(goal_theta+M_PI/2)<0.1745) goal_theta=-M_PI/2;
        else if(fabs(goal_theta-M_PI)<0.1745&&goal_theta>0) goal_theta=M_PI;
        else if(fabs(goal_theta+M_PI)<0.1745&&goal_theta<0) goal_theta=-M_PI;
    }
    else if(flagg==3)//go up
    {
        flag=3;
        goal_theta=abs_theta;
        goal_x=abs_x+1.21;
    }
    else if(flagg==4)//stop
    {
        flag=4;

    }
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"myplan");
    ros::NodeHandle nh;
    t_last=ros::Time::now();
    odom_sub=nh.subscribe("/odom",5,odomCB);
    command=nh.subscribe("/command",1,commandCB);
    control_pub=nh.advertise<geometry_msgs::Twist>("/new_cmd_vel",1);
    reachflag=nh.advertise<std_msgs::String>("/reachflag",1);
    reachOK.data=ok;
    reachflag.publish(reachOK);
    ros::Timer Timer=nh.createTimer(ros::Duration(1/3.0),timeCB);
    ros::Rate r(3.0);
    ROS_INFO_STREAM("wxf");
    ros::spin();
}

