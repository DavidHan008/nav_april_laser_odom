#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<cmath>
#include<algorithm>
#include<ros/timer.h>
#include<nav_msgs/Odometry.h>
#include<std_msgs/String.h>
#include<nav_april_laser_odom/command.h>
#include<nav_april_laser_odom/newodom.h>
double goal_x=0,goal_y=0,goal_theta=0;
double current_x,current_y,current_theta;
ros::Time t_last;
double last_x=0,last_y=0,last_theta=0;
double v_x,v_y,v_theta;
double abs_x,abs_y,abs_theta;
bool sig_;
int state_robot;
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
    bool flag_theta;
    clearTwist();
//同时矫正x

//同时矫正y
//主要跟踪角度theta
        ROS_INFO("goal_theta:%f",goal_theta);
        if(goal_theta==M_PI&&current_theta<0)
        {
            goal_theta=-M_PI;
        }
        else if(goal_theta==-M_PI&&current_theta>0)
        {
            goal_theta=M_PI;
        }
    double dtheta=goal_theta-current_theta;
    if(dtheta>M_PI)
    {
        dtheta-=2*M_PI;
    }
    else if(dtheta<-M_PI)
    {
        dtheta+=2*M_PI;
    }
    ROS_INFO("dtheta is%f",dtheta);

    if(fabs(dtheta)>=theta_tolerance)
    {
        cmdvel.angular.z=std::max(std::min(P_turn*dtheta,v_turnmax),-v_turnmax);
        control_pub.publish(cmdvel);
        flag_theta=false;
    }
    else
    {
        //if(fabs(v_x)<0.001&&fabs(v_y)<0.001&&fabs(v_theta)<0.0005)
        if(sig_)
        {
            ROS_INFO("i am done!");
            flag=4;
            flag_theta=true;
        }
        clearTwist();
        //cmdvel.angular.z=-cmdvel.angular.z*0.15;
        control_pub.publish(cmdvel);
    }
    return flag_theta;
}
bool turnRight()
{
    bool flag_theta;
    clearTwist();
//同时矫正x
//同时矫正y
//主要跟踪角度theta
    ROS_INFO("goal_theta:%f",goal_theta);
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
    if(dtheta>M_PI)
    {
        dtheta-=2*M_PI;
    }
    else if(dtheta<-M_PI)
    {
        dtheta+=2*M_PI;
    }
    if(fabs(dtheta)>=theta_tolerance)
    {
        cmdvel.angular.z=std::max(std::min(P_turn*dtheta,v_turnmax),-v_turnmax);
        control_pub.publish(cmdvel);
        flag_theta=false;
    }
    else
    {
        //if(fabs(v_x)<0.001&&fabs(v_y)<0.001&&fabs(v_theta)<0.0005)
        if(sig_)
        {
            ROS_INFO("i am done!");
            flag=4;
            flag_theta=true;
        }
        clearTwist();
        //cmdvel.angular.z=-cmdvel.angular.z*0.15;
        control_pub.publish(cmdvel);
    }
    return flag_theta;
}
bool goStraight()
{
    double dx=goal_x-current_x;
    ROS_INFO("goal:%f,current_x:%f,dx:%f",goal_x,current_x,dx);
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
        //if(fabs(v_x)<0.001&&fabs(v_y)<0.001&&fabs(v_theta)<0.0005)
        if(sig_)
        {
            ROS_INFO("i am done!");
            ROS_INFO_STREAM(current_x);
            flag=4;
            flag_x=true;
        }
        clearTwist();
        control_pub.publish(cmdvel);
    }
    return flag_x;
}


void odomCB(const nav_april_laser_odom::newodom::ConstPtr msg)
{
    sig_=msg->k;
    tf::Quaternion q;
    q.setW(msg->odom.pose.pose.orientation.w);
    q.setX(msg->odom.pose.pose.orientation.x);
    q.setY(msg->odom.pose.pose.orientation.y);
    q.setZ(msg->odom.pose.pose.orientation.z);
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
        //state_robot=0;
        current_x=msg->odom.pose.pose.position.x;
        current_y=msg->odom.pose.pose.position.y;
    }
    else if(1.3963<current_theta&&current_theta<1.7453)//90 degree
    {
        //state_robot=1;
        current_x=msg->odom.pose.pose.position.y;
        current_y=-msg->odom.pose.pose.position.x;
    }
    else if(2.967<current_theta||current_theta<-2.967)//180 degree
    {
        //state_robot=2;
        current_x=-msg->odom.pose.pose.position.x;
        current_y=-msg->odom.pose.pose.position.y;
    }
    else if(-1.7453<current_theta&&current_theta<-1.3963)//-90 degree
    {
        //state_robot=3;
        current_x=-msg->odom.pose.pose.position.y;
        current_y=msg->odom.pose.pose.position.x;
    }
ROS_INFO("current_theta:%f",current_theta*180/3.1415926);

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
            reachOK.data=ok;
            reachflag.publish(reachOK);
            clearTwist();
            control_pub.publish(cmdvel);
    }
    else
    {
        clearTwist();
        control_pub.publish(cmdvel);
    }
}
void commandCB(const nav_april_laser_odom::command::ConstPtr msg)
{
    firsttime=false;
    //int flagg=atoi((const char * )(msg->data.data()));
    int flagg=msg->command;
    abs_x=current_x;
    abs_y=current_y;
    abs_theta=current_theta;
    ROS_INFO_STREAM(flagg);
    ROS_INFO("abs_x:%f,abs_y:%f,abs_theta:%f",abs_x,abs_y,abs_theta*180/3.1415926);

    if(-0.1745<abs_theta&&abs_theta<0.1745)//0 degree
    {
        state_robot=0;
        goal_x=msg->pose.x;
        goal_y=msg->pose.y;
    }
    else if(1.3963<abs_theta&&abs_theta<1.7453)//90 degree
    {
        state_robot=1;
        goal_x=msg->pose.y;
        goal_y=-msg->pose.x;
    }
    else if(2.967<abs_theta||abs_theta<-2.967)//180 degree
    {
        state_robot=2;
        goal_x=-msg->pose.x;
        goal_y=-msg->pose.y;
    }
    else if(-1.7453<abs_theta&&abs_theta<-1.3963)//-90 degree
    {
        state_robot=3;
        goal_x=-msg->pose.y;
        goal_y=msg->pose.x;
    }

    if(flagg==1)//turn left
    {
        flag=1;
        goal_theta=msg->pose.theta;
    }
    else if(flagg==2)//turn right
    {
        flag=2;
        goal_theta=msg->pose.theta;
    }
    else if(flagg==3)//go up
    {
        flag=3;
        //goal_x=msg->pose.x;
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



