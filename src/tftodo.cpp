#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "tftodo");

  ros::NodeHandle node;

    std::string sourceFrameName = "odom";
    std::string childFrameName = "base_link";
    std::string outputTopicName = "tftodo/odom";
    bool argumentsPassed = false;

    if (argc == 4)
    {
    	sourceFrameName = argv[1];
    	childFrameName = argv[2];
	outputTopicName = argv[3];
	argumentsPassed = true;
    } 

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s and %s.", sourceFrameName.c_str(), childFrameName.c_str(), outputTopicName.c_str());
    }
    else
    {
        ROS_INFO("Got no params, using %s, %s and %s.", sourceFrameName.c_str(), childFrameName.c_str(), outputTopicName.c_str() );
    }

  ros::Publisher odom_publisher = node.advertise<nav_msgs::Odometry>(outputTopicName, 5);

  tf::TransformListener listener;
  ros::Time now = ros::Time::now();

  listener.waitForTransform(sourceFrameName, childFrameName, now, ros::Duration(10.0));

  ros::Rate rate(5.0);
  while (node.ok())
  {
    tf::StampedTransform transform;
    nav_msgs::Odometry odom;
    geometry_msgs::Pose pose;
    try
    {
      now = ros::Time::now();
      listener.lookupTransform(sourceFrameName, childFrameName, ros::Time(0), transform);


    odom.header.stamp = now;
    odom.header.frame_id = sourceFrameName;

    tf::Vector3 currentVector(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    tf::Quaternion currentQuaternion(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());

    tf::Vector3 rotated = currentVector.rotate(currentQuaternion.getAxis(), currentQuaternion.getAngle());

    pose.position.x = rotated.x();
    pose.position.y = rotated.y();
    pose.position.z = rotated.z();
    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    pose.orientation.w = transform.getRotation().w();
    }
    catch (tf::TransformException ex)
    {
      //ROS_ERROR("%s",ex.what());
    }

    
  
    odom.pose.pose = pose;
    odom_publisher.publish(odom);

        ros::spinOnce();
        rate.sleep();  
  }
  return 0;
};
