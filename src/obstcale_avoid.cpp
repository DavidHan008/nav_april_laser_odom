#include"ros/ros.h"
#include"sensor_msgs/LaserScan.h"
#include"std_msgs/String.h"
int flag=0;
ros::Publisher flag_pub;
std_msgs::String ok;
std_msgs::String nook;
void laserCB(const sensor_msgs::LaserScan::ConstPtr scan)
{
    //ROS_INFO("haha");
    flag++;
    if(flag>=10)
    {
        flag=0;
        std::vector<double> readings;
        double sum=0;
        int k=0;

        for(std::vector<float>::const_iterator it = scan->ranges.begin();
          it != scan->ranges.end();
          ++it)
        {
            k++;
          //readings.push_back(*it);
            if(k>270&&k<450)
            {
                if((*it)<0.8)
                    sum=sum+1;
            }
        }
        if(sum/180>0.2)
        {
            ROS_INFO("There is an obstcale!");
            flag_pub.publish(nook);
        }
        else
        {
            ROS_INFO("It is safe now");
            flag_pub.publish(ok);
        }
        //ROS_INFO_STREAM(sum/180);

    }
}
int main(int argc, char **argv)
{
	ros::init(argc,argv, "obstcale_avoid");
    ros::NodeHandle nh;
    ros::Subscriber laser_sub=nh.subscribe("/scan",1,laserCB);
    flag_pub=nh.advertise<std_msgs::String>("obstcale",5);
    ok.data="1";
    nook.data="0";
    ros::spin();
}

