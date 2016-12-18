#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
int main(int argc,char** argv)
{
	ros::init(argc,argv,"fake_topic_pub");
    ros::NodeHandle nh;
    ros::Publisher Pub_correctOdom=nh.advertise<nav_msgs::Odometry>("correctOdom",5,true);
    tf::TransformBroadcaster pub_tf;
    ros::Rate r(20.0);
    tf::StampedTransform base2odom_tf(tf::Transform(tf::Quaternion(0,0,0.7071,0.7071),tf::Vector3(0.5,1.5,0)),ros::Time::now(),"odom","base_link");
    tf::StampedTransform odom2map_tf(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(-0.0,0.25,0)),ros::Time::now(),"map","odom");
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x=0.5;
    odom.pose.pose.position.y=1.75;
    odom.pose.pose.position.z=0.0;
    odom.pose.pose.orientation.w=0.7071;
    odom.pose.pose.orientation.z=0.7071;
    odom.pose.pose.orientation.y=0.0;
    odom.pose.pose.orientation.x=0.0;
    while(nh.ok())
    {
        odom.header.stamp=ros::Time::now();
        base2odom_tf.stamp_=ros::Time::now();
        odom2map_tf.stamp_=ros::Time::now();
        odom.pose.pose.position.x+=0.005;
        base2odom_tf.setOrigin(tf::Vector3(base2odom_tf.getOrigin().getX()+0.005,base2odom_tf.getOrigin().getY(),base2odom_tf.getOrigin().getZ()));
        pub_tf.sendTransform(base2odom_tf);
        pub_tf.sendTransform(odom2map_tf);
        Pub_correctOdom.publish(odom);
        r.sleep();
    }
}
