/***

 */
#include<string>
#include<sstream>
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include<std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include<tf/transform_broadcaster.h>
#include<tf/tf.h>
#include<tf/tfMessage.h>
#include<nav_msgs/Odometry.h>
#include<stdio.h>
#include<geometry_msgs/PoseStamped.h>
serial::Serial ser;
const char *d=" ";
float L=0.2897;
//在ros收到string消息，往串口写。
void write_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    float vx=msg->linear.x;
    float vy=msg->linear.y;
    float vangle=msg->angular.z;
    float vR,vL;
    char simb_R,simb_L;
    char buffer[16];
    if (vy==0.0)
    {
        if(vx==0)
        {
            vR=vangle*L/2;
            vL=-vR;
        }
        else if(vangle==0)
        {
            vR=vx;
            vL=vx;
        }
        else
        {
            vR=vx+vangle*L/2;
            vL=vx-vangle*L/2;
        }
      if(vR>0)
          simb_R='+';
      else
          simb_R='-';
      if(vL>0)
          simb_L='+';
      else
          simb_L='-';
      int iR=abs((int)vR),iL=abs((int)vL);
      int dR=(int)((fabs(vR)-iR)*1000+0.5),dL=(int)((fabs(vL)-iL)*1000+0.5);
      sprintf(buffer,"%c%02d.%03d,%c%02d.%03d",simb_L,iL,dL,simb_R,iR,dR);
      //ROS_INFO_STREAM(buffer);
        ser.write(buffer);
    }
    //ser.write(msg->data);

}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_by_keyboard");
    ros::NodeHandle nh;
    //将ros的“write”主题发送给串口
 ros::Subscriber write_sub = nh.subscribe("/cmd_vel_mux/input/teleop", 10, write_callback);
   //ros::Subscriber write_sub = nh.subscribe("/cmd_vel", 1000, write_callback);
    //将封装好的odom信息发布出去
    ros::Publisher odom_pub=nh.advertise<nav_msgs::Odometry>("odomInit",50);
    ros::Publisher pose_pub=nh.advertise<geometry_msgs::PoseStamped>("odom_init",20);
    static tf::TransformBroadcaster tf_pub;
    ros::Time currentTime;
    //加入全局变量positionLast,防止位置发送突变
    float xLast=0,yLast=0,yawLast=0;
    //for test:
   //float xLast=0.33,yLast=0.026,yawLast=0.74;
//串口初始化
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(2000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
//字符串分割初始化
    const char *sep=" ";
    char *p;
    float Position[100];
    std::string wrongResult;
//
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        int index=ser.available();
        //ROS_INFO_STREAM(index);
    //有效串口数据（string转float）
        if(index==24)
        {
            std::string s;
            s= ser.readline();
            ROS_INFO_STREAM(s);
            char * ss=(char*)s.data();
            p=strtok(ss,sep);
            Position[0]=atof(p);
            //ROS_INFO_STREAM(Position[0]);
            for(int i=1;i<3;i++)
            {
                p=strtok(NULL,sep);
                Position[i]=atof(p);
                //ROS_INFO_STREAM(Position[i]);
            }
            //判断数据变化量是否满足要求，防止数据突变
            if (fabs(xLast-Position[0])> 0.1||fabs(yLast-Position[1])>0.1||fabs(yawLast-Position[2])>0.5){;}
            else{
             //满足：更新全局变量
                        xLast=Position[0];
                        yLast=Position[1];
                        yawLast=Position[2];
                        //里程计信息封装并发布
                        geometry_msgs::PoseStamped pose_odom;
                        geometry_msgs::Quaternion odom_quat=tf::createQuaternionMsgFromYaw(Position[2]);
                        tf::Quaternion Q=tf::createQuaternionFromYaw(Position[2]);
                        nav_msgs::Odometry odom;
                        tf::Transform transform;
                        odom.header.stamp=ros::Time::now();
                        odom.header.frame_id="odom";
                        odom.pose.pose.position.x=Position[0];
                        odom.pose.pose.position.y=Position[1];
                        odom.pose.pose.position.z=0;
                        odom.pose.pose.orientation=odom_quat;
                        odom.child_frame_id="base_link";
                        odom_pub.publish(odom);
                        //ROS_INFO_STREAM("HAHA");
                        tf::Vector3 origin(Position[0],Position[1],0);
                        transform.setRotation(Q);
                        transform.setOrigin(origin);
                        tf_pub.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","base_link" ));
                        //************************
                        pose_odom.pose.orientation=odom.pose.pose.orientation;
                        pose_odom.pose.position=odom.pose.pose.position;
                        pose_pub.publish(pose_odom);
                    }
        }
    //无效串口数据（释放）
        else if(index)
        {
            //ROS_INFO_STREAM(ser.available());
             wrongResult= ser.readline();
             ROS_INFO_STREAM(wrongResult);
        }
        loop_rate.sleep();

    }
}

