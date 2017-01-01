#include <apriltags_ros/apriltag_detector_nav.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
//#include <apriltags_ros/AprilTagDetection.h>
//#include <apriltags_ros/AprilTagDetectionArray.h>
#include<nav_april_laser_odom/PoseStampedArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>
#include<eigen3/Eigen/Eigen>
#include<Eigen/Eigen>
#include<tf2_eigen/tf2_eigen.h>
#include<std_msgs/String.h>
namespace apriltags_ros{

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh){

  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
   sensor_frame_id_ = "";
  }
  /*
  //convet camera to world frame
  tf::Matrix3x3 m2;
  //m2.setValue(0,1,0,1,0,0,0,0,-1);
  m2.setValue(0,-1,0, -1,0,0,0,0,-1);
  camera2world.setIdentity();
  camera2world.setBasis(m2);
  camera2world=camera2world.inverse();
  */
  //convet tag to world frame
  tf::Matrix3x3 m0;
  m0.setValue(0,1,0,
                       -1,0,0,
                       0,0,1);
  tag2world.setIdentity();;
  tag2world.setBasis(m0);
  //convert camera to base
  tf::Matrix3x3 m2;
  m2.setValue(0,-1,0,
                        -1,0,0,
                        0,0,-1);
  base2camera.setIdentity();
  base2camera.setBasis(m2);
  base2camera.setOrigin(tf::Vector3(0.005,0.005,0));
  //*************************************************
  AprilTags::TagCodes tag_codes = AprilTags::tagCodes36h11;
  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(tag_codes));
  image_sub_ = it_.subscribe("image_rect", 1, &AprilTagDetector::imageCb, this);
  //wxf:add for init transform of map and world
  map2world.setIdentity();
  //***********************************************************************************************************
  image_pub_ = it_.advertise("tag_detections_image", 1);
  id_pub=nh.advertise<std_msgs::String>("tag_id",1);
}
AprilTagDetector::~AprilTagDetector(){
  image_sub_.shutdown();
}
/*
void AprilTagDetector::OdomCb(const nav_msgs::OdometryConstPtr &msg)
{
    turtle_odom.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    turtle_odom.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
ROS_INFO("first odom(x:%f,y:%f)",msg->pose.pose.position.x,msg->pose.pose.position.y);
}
*/
void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //ROS_INFO_STREAM("haha");
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());
//ROS_INFO("%d tag detected", (int)detections.size());

  sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo);
  boost::array<double,9> param={691.743632581011, 0.0, 376.5170843870497, 0.0, 691.7146437795241, 250.7975382456949, 0.0, 0.0, 1.0};
  cam_info->K =  param;
  double fx = cam_info->K[0];
  double fy = cam_info->K[4];
  double px = cam_info->K[2];
  double py = cam_info->K[5];

  if(!sensor_frame_id_.empty())
    cv_ptr->header.frame_id = sensor_frame_id_;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }
    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();
    stringstream stream;
    stream<<description.id();
    string id_str;
    stream>>id_str;
    //  ROS_INFO("%lf",tag_size);

    //detection.draw(cv_ptr->image);
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0,0,3,3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);
//WXF:add
    //first:get tag pose according to the world
    tf::Vector3 orig(description.getx(),description.gety(),description.getz());
    tag2world.setOrigin(orig);
    //ROS_INFO("this tag's location is(x:%.3f, y:%.3f. z:%.3f,,,,,x:%.3f, y:%.3f. z:%.3f w:%.3f) ",TagPose.getOrigin().getX(),TagPose.getOrigin().getY(),TagPose.getOrigin().getZ(),TagPose.getRotation().getX(),TagPose.getRotation().getY(),TagPose.getRotation().getZ(),TagPose.getRotation().getW());
    //second:获取camera和tag的相对tf信息
    //get the transform between tag and camera
    tf::Transform camera2tag_tf,camera2world_tf,tag2camera_tf,base2world;
    tf::Quaternion Q(rot_quaternion.x(),rot_quaternion.y(),rot_quaternion.z(),rot_quaternion.w());
    tf::Vector3 origin(transform(0,3),transform(1,3),transform(2,3));
    tag2camera_tf.setRotation(Q);
    tag2camera_tf.setOrigin(origin);
   //camera2tag_tf:camera pose according to tag
    camera2tag_tf=tag2camera_tf.inverse();
    camera2tag_tf.setOrigin(tf::Vector3(camera2tag_tf.getOrigin().getX(),camera2tag_tf.getOrigin().getY(),camera2tag_tf.getOrigin().getZ()));
    //thid:get the pose of camera inthe world frame
    camera2world_tf.mult(tag2world,camera2tag_tf);
    double _yew_camera,_pich_camera,_roll_camera;
    double _yew_tag,_pich_tag,_roll_tag;
    double _yew_world,_pich_world,_roll_world;
    camera2world_tf.getBasis().getEulerZYX(_yew_world,_pich_world,_roll_world);
    //_yew_world=_yew_world+M_PI/2;
    //_pich_world=0;_roll_world=0;
    if(_yew_world>M_PI)
    {
        _yew_world-=2*M_PI;
    }
    else if(_yew_world<-M_PI)
    {
        _yew_world+=2*M_PI;
    }
    tf::Quaternion qq=tf::createQuaternionFromRPY(_roll_world,_pich_world,_yew_world);
    //tf::Quaternion qq=tf::createQuaternionFromYaw(_yew_world);
    camera2world_tf.setRotation(qq);
    //print for debug
    camera2tag_tf.getBasis().getEulerZYX(_yew_camera,_pich_camera,_roll_camera);
    tag2camera_tf.getBasis().getEulerZYX(_yew_tag,_pich_tag,_roll_tag);

    ROS_INFO("camrea2world_tf is(x:%.3f, y:%.3f. z:%.3f,,,,,angle:%.3f,%.3f,%.3f) ",
             camera2world_tf.getOrigin().getX(),camera2world_tf.getOrigin().getY(),camera2world_tf.getOrigin().getZ(),_yew_world*180/3.1415926,_pich_world*180/3.1415926,_roll_world*180/3.1415926);

//forth:publish the tf of robot according to world
    //tf::StampedTransform base2world(camera2world_tf,ros::Time::now(),"world","base_link");
    //tf_pub_.sendTransform(base2world);
    //get the map pose in the world frame
    //tf::StampedTransform base2odom,odom2map;

    bool look_tf_mark=lookfor_tf();
    //test
    //look_tf_mark=true;
    if(look_tf_mark)
    {      
        double _yew_test3,_pich_test3,_roll_test3;
        base2odom.getBasis().getEulerZYX(_yew_test3,_pich_test3,_roll_test3);
        /*
        ROS_INFO("base2odom111 is(x:%.3f, y:%.3f. z:%.3f,,,,,angle:%.3f,%.3f,%.3f) ",
                 base2odom.getOrigin().getX(),base2odom.getOrigin().getY(),base2odom.getOrigin().getZ(),
                 _yew_test3*180/3.1415926,_pich_test3*180/3.1415926,_roll_test3*180/3.1415926);
*/
        //map2world.mult(camera2world_tf,temp.inverse());
        //camera2base.setIdentity();
        //camera2base.setOrigin(tf::Vector3(0.05,0,0));
        base2world.mult(camera2world_tf,base2camera);
        //base2world=camera2world_tf;
        double _yew_test4,_pich_test4,_roll_test4;
        base2world.getBasis().getEulerZYX(_yew_test4,_pich_test4,_roll_test4);
        ROS_INFO("base2world is(x:%.3f, y:%.3f. z:%.3f,,,,,angle:%.3f,%.3f,%.3f) ",
                 base2world.getOrigin().getX(),base2world.getOrigin().getY(),base2world.getOrigin().getZ(),
                 _yew_test4*180/3.1415926,_pich_test4*180/3.1415926,_roll_test4*180/3.1415926);

        odom2world.mult(base2world,base2odom.inverse());
        double _yew_test2,_pich_test2,_roll_test2;
        //map2world.getBasis().getEulerZYX(_yew_test2,_pich_test2,_roll_test2);
        odom2world.getBasis().getEulerZYX(_yew_test2,_pich_test2,_roll_test2);
        _pich_test2=0;_roll_test2=0;
        tf::Quaternion qq=tf::createQuaternionFromYaw(_yew_test2);
        odom2world.setRotation(qq);
        odom2world.setOrigin(tf::Vector3(odom2world.getOrigin().getX(),odom2world.getOrigin().getY(),0));
/*
        ROS_INFO("odom2world is(x:%.3f, y:%.3f. z:%.3f,,,,,angle:%.3f,%.3f,%.3f) ",
                 odom2world.getOrigin().getX(),odom2world.getOrigin().getY(),odom2world.getOrigin().getZ(),
                 _yew_test2*180/3.1415926,_pich_test2*180/3.1415926,_roll_test2*180/3.1415926);
*/
        //tf_pub_.sendTransform(tf::StampedTransform(map2world,ros::Time::now(),"world","map"));
        tf_pub_.sendTransform(tf::StampedTransform(odom2world,ros::Time::now(),"world","odom"));
    }
    else
    {
        ROS_WARN("i use the old tf");
        tf_pub_.sendTransform(tf::StampedTransform(odom2world,ros::Time::now(),"world","odom"));
    }
  }

}
bool AprilTagDetector::lookfor_tf()
{
    try{
        tf_listener.lookupTransform("odom","base_link",ros::Time(0),base2odom);
    }
    catch(tf::TransformException ex){
            ROS_ERROR("WXF1:%s",ex.what());
            return false;
    }
    /*
    try{
        tf_listener.lookupTransform("map","odom",ros::Time(0),odom2map);
    }
    catch(tf::TransformException ex){
            ROS_ERROR("WXF2:%s",ex.what());
            return false;
    }
    */
    return true;
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    //wxf:add
    ROS_ASSERT(tag_description["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(tag_description["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    ROS_ASSERT(tag_description["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    double x = (double)tag_description["x"];
    //x=x+deltaxx;//consider the transform between camera and base
    double y = (double)tag_description["y"];
    double z = (double)tag_description["z"];
    //************************************************8
    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name,x,y,z);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name<<"x:"<<x<<"y:"<<y<<"z:"<<z);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}


}
