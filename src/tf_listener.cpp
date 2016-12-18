#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
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
private:
    tf::StampedTransform map2world;
    void odomCB(const nav_msgs::OdometryConstPtr msg);
};
tf_listener::tf_listener()
{
    map2world.setIdentity();
    odom_pub=nh.advertise<nav_msgs::Odometry>("odom",5);
    odom_sub=nh.subscribe("correctOdom",5,&tf_listener::odomCB,this);
}
void tf_listener::odomCB(const nav_msgs::OdometryConstPtr msg)
{
    try{
        listener.lookupTransform("world","map",ros::Time(0),map2world);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("WXF3:%s",ex.what());
    }
    tf::Transform odom2pose,base2world_tf;
    odom2pose.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    odom2pose.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
    base2world_tf.mult(map2world,odom2pose);
    nav_msgs::Odometry base2world_odom;
    base2world_odom.header.frame_id="world";
    base2world_odom.header.stamp=ros::Time::now();
    base2world_odom.child_frame_id="base_link";
    base2world_odom.pose.pose.position.x=base2world_tf.getOrigin().getX();
    base2world_odom.pose.pose.position.y=base2world_tf.getOrigin().getY();
    base2world_odom.pose.pose.position.z=base2world_tf.getOrigin().getZ();
    base2world_odom.pose.pose.orientation.w=base2world_tf.getRotation().getW();
    base2world_odom.pose.pose.orientation.x=base2world_tf.getRotation().getX();
    base2world_odom.pose.pose.orientation.y=base2world_tf.getRotation().getY();
    base2world_odom.pose.pose.orientation.z=base2world_tf.getRotation().getZ();
    double _yew_test2,_pich_test2,_roll_test2;
    base2world_tf.getBasis().getEulerZYX(_yew_test2,_pich_test2,_roll_test2);

    ROS_INFO("result is(x:%.3f, y:%.3f. z:%.3f,,,,,angle:%.3f,%.3f,%.3f) ",
             base2world_tf.getOrigin().getX(),base2world_tf.getOrigin().getY(),base2world_tf.getOrigin().getZ(),
             _yew_test2*180/3.1415926,_pich_test2*180/3.1415926,_roll_test2*180/3.1415926);
    odom_pub.publish(base2world_odom);
    tf_pub.sendTransform(tf::StampedTransform(base2world_tf,ros::Time::now(),"world1","base_link1"));

}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"tf_listener");
    tf_listener haha;
    ros::spin();
    return 0;
}
