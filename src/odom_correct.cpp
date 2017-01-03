#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<nav_april_laser_odom/newodom.h>
class tf_listener
{
public:
    tf_listener();
private:
    ros::NodeHandle nh;
    tf::TransformListener listener;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tf_pub;
    ros::Subscriber odom_sub;
    ros::Time t_last=ros::Time::now();
    double last_x=0,last_y=0,last_theta=0;
private:
    tf::StampedTransform map2world,odom2world;
    void odomCB(const nav_msgs::OdometryConstPtr msg);
};
tf_listener::tf_listener()
{
    map2world.setIdentity();
    //odom_pub=nh.advertise<nav_msgs::Odometry>("odom",5);
    odom_pub=nh.advertise<nav_april_laser_odom::newodom>("odom",5);
    //odom_sub=nh.subscribe("correctOdom",5,&tf_listener::odomCB,this);
    odom_sub=nh.subscribe("odomInit",5,&tf_listener::odomCB,this);
}
void tf_listener::odomCB(const nav_msgs::OdometryConstPtr msg)
{
    try{
        listener.lookupTransform("world","odom",ros::Time(0),odom2world);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("WXF3:%s",ex.what());
    }
    tf::Transform base2odom,base2world_tf;
    base2odom.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    base2odom.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
       //TEST
    double _yew_test,_pich_test,_roll_test;
    base2odom.getBasis().getEulerZYX(_yew_test,_pich_test,_roll_test);
/*
    ROS_INFO("base2odom is(x:%.3f, y:%.3f. z:%.3f,,,,,angle:%.3f,%.3f,%.3f) ",
             base2odom.getOrigin().getX(),base2odom.getOrigin().getY(),base2odom.getOrigin().getZ(),
             _yew_test*180/3.1415926,_pich_test*180/3.1415926,_roll_test*180/3.1415926);
*/
    double _yew_test1,_pich_test1,_roll_test1;
    odom2world.getBasis().getEulerZYX(_yew_test1,_pich_test1,_roll_test1);
    /*
    ROS_INFO("odom2world is(x:%.3f, y:%.3f. z:%.3f,,,,,angle:%.3f,%.3f,%.3f) ",
             odom2world.getOrigin().getX(),odom2world.getOrigin().getY(),odom2world.getOrigin().getZ(),
             _yew_test1*180/3.1415926,_pich_test1*180/3.1415926,_roll_test1*180/3.1415926);
	*/
    base2world_tf.mult(odom2world,base2odom);

    double _yew_test2,_pich_test2,_roll_test2;
    base2world_tf.getBasis().getEulerZYX(_yew_test2,_pich_test2,_roll_test2);
    tf::Quaternion qq=tf::createQuaternionFromYaw(_yew_test2+0.0175);
   //tf::Quaternion qq=tf::createQuaternionFromYaw(_yew_test2);
    base2world_tf.setRotation(qq);

    nav_msgs::Odometry base2world_odom;
    base2world_odom.header.frame_id="world2";
    base2world_odom.header.stamp=ros::Time::now();
    base2world_odom.child_frame_id="base_link1";
    base2world_odom.pose.pose.position.x=base2world_tf.getOrigin().getX();
    base2world_odom.pose.pose.position.y=base2world_tf.getOrigin().getY();
    base2world_odom.pose.pose.position.z=base2world_tf.getOrigin().getZ();
    base2world_odom.pose.pose.orientation.w=base2world_tf.getRotation().getW();
    base2world_odom.pose.pose.orientation.x=base2world_tf.getRotation().getX();
    base2world_odom.pose.pose.orientation.y=base2world_tf.getRotation().getY();
    base2world_odom.pose.pose.orientation.z=base2world_tf.getRotation().getZ();
  
    ROS_INFO("result is(x:%.3f, y:%.3f. z:%.3f,,,,,angle:%.3f,%.3f,%.3f) ",
             base2world_tf.getOrigin().getX(),base2world_tf.getOrigin().getY(),base2world_tf.getOrigin().getZ(),
             (_yew_test2+0.0175)*180/3.1415926,_pich_test2*180/3.1415926,_roll_test2*180/3.1415926);    

    nav_april_laser_odom::newodom newodom;
    newodom.odom=base2world_odom;
    ros::Time t_now=ros::Time::now();
    ros::Duration dt=t_now-t_last;
    bool sig_;
    if(fabs(base2odom.getOrigin().getX()-last_x)/dt.toSec()<0.002&&fabs(base2odom.getOrigin().getY()-last_y)/dt.toSec()<0.002&&fabs(_yew_test-last_theta)/dt.toSec()<0.0005)
        sig_=true;
    else
        sig_=false;
    newodom.k=sig_;
    odom_pub.publish(newodom);
    tf_pub.sendTransform(tf::StampedTransform(base2world_tf,ros::Time::now(),"world2","base_link1"));
    last_x=base2odom.getOrigin().getX();
    last_y=base2odom.getOrigin().getY();
    last_theta=_yew_test;
    t_last=t_now;
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"odom_correct");
    tf_listener haha;
    ros::spin();
    return 0;
}
