#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include<std_msgs/String.h>
#include<string>
#include<sstream>
ros::Publisher cmdVelPub;
ros::Publisher marker_pub;
ros::Subscriber id_sub;
int id;
bool myflag=false;
//tag是相对于世界坐标系的存在，由于tag和相机位置可以直接得到，而相机和base还存在一个offset，所以这边要加入一个修正量
double deltaxy=0.0;
void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());
  ros::Duration(1).sleep(); // sleep for  a second
  ROS_INFO("nav_square.cpp ended!");
  ros::shutdown();
}

void init_markers(visualization_msgs::Marker *marker)
{
  marker->ns       = "waypoints";
  marker->id       = 0;
  marker->type     = visualization_msgs::Marker::CUBE_LIST;
  marker->action   = visualization_msgs::Marker::ADD;
  marker->lifetime = ros::Duration();//0 is forever
  marker->scale.x  = 0.2;
  marker->scale.y  = 0.2;
  marker->color.r  = 1.0;
  marker->color.g  = 0.7;
  marker->color.b  = 1.0;
  marker->color.a  = 1.0;

  marker->header.frame_id = "world2";
  marker->header.stamp = ros::Time::now();

}
//add for receieve tagID
void idCB(const std_msgs::StringConstPtr msg)
{
    char*  id_str;
    id_str=(char *)(msg->data.data());
    id=atoi(id_str);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "movesquare");
  std::string topic = "/cmd_vel";
  ros::NodeHandle node;
  //get tagID
  //id_sub=node.subscribe("tag_id",1,idCB);
  //Subscribe to the move_base action server
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

  //Define a marker publisher.
  marker_pub = node.advertise<visualization_msgs::Marker>("waypoint_markers", 10);

  //for init_markers function
  visualization_msgs::Marker  line_list;

  signal(SIGINT, shutdown);
  ROS_INFO("move_base_square.cpp start...");

  //How big is the square we want the robot to navigate?
  double square_size = 0.6;

  //Create a list to hold the target quaternions (orientations)
  geometry_msgs::Quaternion quaternions[4];

  //convert the angles to quaternions
  double angle = -M_PI/2;
  int angle_count = 0;
  for(angle_count = 0; angle_count < 4;angle_count++ )
  {
      quaternions[angle_count] = tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
      angle = angle + M_PI/2;
  }

  //a pose consisting of a position and orientation in the map frame.
  geometry_msgs::Point point;
  geometry_msgs::Pose pose_list[5];
  geometry_msgs::Pose pose_listNew[9];
  //int nn=sizeof(pose_list)/sizeof(pose_list[0]);
  //int mm=sizeof(pose_listNew)/sizeof(pose_listNew[0]);
  //int id2num[9]={0,4,8,9,10,6,2,1,0};
  //first point
  point.x = -0.00;
  point.y = 0.0;
  point.z = 0.0;
  pose_list[0].position = point;
  pose_list[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  //first new point
  pose_listNew[0].position = point;
  pose_listNew[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

//second point
  point.x = square_size*2-0.01;
  point.y = 0.0;
  point.z = 0.0;
  pose_list[1].position = point;
  pose_list[1].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI/2);
  //second1 new point
  pose_listNew[1].position = point;
  pose_listNew[1].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  //second2 new point
  pose_listNew[2].position = point;
  pose_listNew[2].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI/2);
//third point
  point.x=square_size*2;
  point.y=-square_size*2;
  point.z = 0.0;
  pose_list[2].position = point;
  pose_list[2].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI);
//third1 new point
  pose_listNew[3].position = point;
  pose_listNew[3].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI/2);
//third2 new point
  pose_listNew[4].position = point;
  pose_listNew[4].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI);
//forth point
  point.x=0.0;
  point.y=-square_size*2;
  point.z = 0.0;
  pose_list[3].position = point;
  pose_list[3].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI*1.5);
  //forth1 new point
  pose_listNew[5].position = point;
  pose_listNew[5].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI);
  //forth2 new point
  pose_listNew[6].position=point;
  pose_listNew[6].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI*1.5);
//fifth point
  point.x=0.0;
  point.y=0.0;
  point.z = 0.0;
  pose_list[4].position = point;
  pose_list[4].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI*2);
//fifth1 new point
  pose_listNew[7].position = point;
  pose_listNew[7].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI*1.5);
  //fifth2 new point
  pose_listNew[8].position = point;
  pose_listNew[8].orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-M_PI*2);
  //Initialize the visualization markers for RViz
  init_markers(&line_list);

  //Set a visualization marker at each waypoint
  for(int i = 0; i < 9; i++)
  {
      //**************************************
    //line_list.points.push_back(pose_list[i].position);
      line_list.points.push_back(pose_listNew[i].position);
    //******************************************
  }
marker_pub.publish(line_list);
  //Publisher to manually control the robot (e.g. to stop it, queue_size=5)
  cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 5);



  ROS_INFO("Waiting for move_base action server...");
  //Wait 60 seconds for the action server to become available
  if(!ac.waitForServer(ros::Duration(60)))
  {
    ROS_INFO("Can't connected to move base server");
    return 1;
  }
  ROS_INFO("Connected to move base server");
  ROS_INFO("Starting navigation test");

  //Initialize a counter to track waypoints
  int count = 1;
  //Cycle through the four waypoints
  while( (count < 9) && (ros::ok()) )
  {
     //Update the marker display
     marker_pub.publish(line_list);

     //Intialize the waypoint goal
     move_base_msgs::MoveBaseGoal goal;

     //Use the map frame to define goal poses
     goal.target_pose.header.frame_id = "world2";
     //goal.target_pose.header.frame_id = "odom";

     //Set the time stamp to "now"
     goal.target_pose.header.stamp = ros::Time::now();

     //Set the goal pose to the i-th waypoint
     //******************************************
     //goal.target_pose.pose = pose_list[count];
     goal.target_pose.pose = pose_listNew[count];
    //********************************************
     //Start the robot moving toward the goal
     //Send the goal pose to the MoveBaseAction server
     ac.sendGoal(goal);

    //Allow 3 minute to get there
    bool finished_within_time = ac.waitForResult(ros::Duration(180));

    //If we dont get there in time, abort the goal
    if(!finished_within_time)
    {
        ac.cancelGoal();
        ROS_INFO("Timed out achieving goal");
    }
    else
    {
        //We made it!
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {

            if(!myflag)
            {
                count=count-1;
               myflag=true;
            }
            else
            {
                myflag=false;
                ROS_INFO("Goal succeeded!");
            }
        }
        else
        {
            ROS_INFO("The base failed for some reason");
        }
    }
     count += 1;
  }
  ROS_INFO("move_base_square.cpp end...");
  return 0;
}
