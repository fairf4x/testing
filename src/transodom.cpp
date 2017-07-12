#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <string>

class Transodom
{
    public:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    { 
	//ROS_INFO("I heard: [%f,%f,%f]", odom->pose.pose.position.x, odom->pose.pose.position.y, 
	//				odom->pose.pose.position.z);

	toSend.header.seq = ++count;
	toSend.header.stamp = odom->header.stamp;
	toSend.header.frame_id = "odom";

	geometry_msgs::PoseWithCovariance poses;
	poses = odom->pose;

	toSend.pose = poses;
	toSend = Transform(toSend);
	newMessage = true;
    }

    void initPublisherAndSubscriber(ros::NodeHandle n)
    {
	initSubscriber(n, "/odom_to_transform");
        initPublisher(n, "/transformed_odom");
    }

    void initPublisherAndSubscriber(ros::NodeHandle n, std::string sourceTopicName, std::string outputTopicName)
    {
	initSubscriber(n, sourceTopicName);
        initPublisher(n, outputTopicName);
    }

    Transodom(tf::Vector3 tra, tf::Vector3 rot, tf::Vector3 sca) 
    { 
        newMessage = false;
	count = 0;

	translation = tra;
	rotation = rot;
	scale = sca;
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    odometryPublisher.publish(toSend);
	    newMessage = false;
	}
    }

    private:
    ros::Publisher odometryPublisher;
    ros::Subscriber odometrySubscriber;

    nav_msgs::Odometry toSend;
    bool newMessage;
    int count;

    tf::Vector3 translation;
    tf::Vector3 rotation;
    tf::Vector3 scale;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        odometrySubscriber = n.subscribe(sourceTopicName, 50, &Transodom::odomCallback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        odometryPublisher = n.advertise<nav_msgs::Odometry>(outputTopicName, 50); 
    }

    nav_msgs::Odometry Transform(nav_msgs::Odometry toTransform)
    {
	nav_msgs::Odometry toReturn;
	toReturn.header = toTransform.header;
	toReturn.pose = toTransform.pose;

/*
	tf::Vector3 axis(0,0,1);
	int permutation[3] = {1, 3, 2};
	double angle = 2.1;
	double xScale = 8;
	double yScale = 8;
	double zScale = 2;

	tf::Vector3 working(toTransform.pose.pose.position.x, toTransform.pose.pose.position.y, toTransform.pose.pose.position.z);
	working = Permute(working, permutation);
	working = working.rotate(axis, angle);

	toReturn.pose.pose.position.x = working.getX() + 0.6;	
	toReturn.pose.pose.position.y = working.getY() - 0.35;
	toReturn.pose.pose.position.z = working.getZ() + 1.65;
*/
	tf::Vector3 working(toTransform.pose.pose.position.x, toTransform.pose.pose.position.y, toTransform.pose.pose.position.z);

	working = working.rotate(tf::Vector3(1,0,0), rotation.getX());
	working = working.rotate(tf::Vector3(0,1,0), rotation.getY());
	working = working.rotate(tf::Vector3(0,0,1), rotation.getZ());

	toReturn.pose.pose.position.x = working.getX() + translation.getX();	
	toReturn.pose.pose.position.y = working.getY() + translation.getY();
	toReturn.pose.pose.position.z = working.getZ() + translation.getZ();

	return toReturn;
    }

    tf::Vector3 Permute(tf::Vector3 toPermute, int permutation[])
    {
	tf::Vector3 toReturn;
	toReturn.setX(permutationValue(toPermute, permutation[0]));
	toReturn.setY(permutationValue(toPermute, permutation[1]));
	toReturn.setZ(permutationValue(toPermute, permutation[2]));
	
	return toReturn;
    }

    double permutationValue(tf::Vector3 source, int element)
    {
	if (element == 1)
	    return source.getX();

	if (element == 2)
	    return source.getY();

	return source.getZ();
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "transodom");
    ros::NodeHandle n;

    ROS_INFO("Have %i arguments passed.", argc);

    std::string sourceTopicName;
    std::string outputTopicName;
    bool argumentsPassed = false;

    if (argc == 3)
    {
    	sourceTopicName = argv[1];
	outputTopicName = argv[2];
	argumentsPassed = true;
    } 

    double xTranslation;
    double yTranslation;
    double zTranslation;
    double xAxisRotation;
    double yAxisRotation;
    double zAxisRotation;
    double xScale;
    double yScale;
    double zScale;

    n.param<double>("dso_base_tr_x_translation", xTranslation, 0.0);
    n.param<double>("dso_base_tr_y_translation", yTranslation, 0.0);
    n.param<double>("dso_base_tr_z_translation", zTranslation, 0.0);
    n.param<double>("dso_base_tr_x_rotation", xAxisRotation, 0.0);
    n.param<double>("dso_base_tr_y_rotation", yAxisRotation, 0.0);
    n.param<double>("dso_base_tr_z_rotation", zAxisRotation, 0.0);
    n.param<double>("dso_base_tr_x_scale", xScale, 1.0);
    n.param<double>("dso_base_tr_y_scale", yScale, 1.0);
    n.param<double>("dso_base_tr_z_scale", zScale, 1.0);

    Transodom transodo( tf::Vector3(xTranslation, yTranslation, zTranslation), 
			tf::Vector3(xAxisRotation, yAxisRotation, zAxisRotation), 
			tf::Vector3(xScale, yScale, zScale));

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	transodo.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /odom_to_transform and /transformed_odom.");
	transodo.initPublisherAndSubscriber(n);
    }

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        transodo.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
