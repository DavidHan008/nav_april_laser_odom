#include<ros/ros.h>
#include<std_msgs/String.h>
char plans[]={'3','2','3','2','3','2','3'};
int i=0;
ros::Publisher command;
std_msgs::String path;
void reachCB(const std_msgs::String msg)
{
    if(i<7)
    {
        ROS_INFO("GET THE PATH");
        path.data=plans[i];
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
    command=nh.advertise<std_msgs::String>("command",1);

    ros::spin();

}

