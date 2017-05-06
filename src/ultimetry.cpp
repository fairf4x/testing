#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <string>

class OdometrySubscriber
{
    public:
    void callback(const nav_msgs::Odometry::ConstPtr& odom)
    {
	nav_msgs::Odometry newOdometry;

	odometry = newOdometry;
    }

    nav_msgs::Odometry GetNewest()
    {
	return odometry;
    }

    void initSubscriber(ros::NodeHandle n, std::string odometryTopicName)
    {
        subscriber = n.subscribe(odometryTopicName, 50, &OdometrySubscriber::callback, this);
    }

    private:
    ros::Subscriber subscriber;
    nav_msgs::Odometry odometry;
};

class Ultimetry
{
    public:
    void droneCommandCallback(const nav_msgs::Odometry::ConstPtr& cmd)
    { 
	ROS_INFO("I heard: [%s]", cmd->header.frame_id.c_str());
    }

    void initPublisherAndSubscribers(ros::NodeHandle n)
    {
	droneOdometrySubscriber.initSubscriber(n, "/bebop/odom");
	dsoOdometrySubscriber.initSubscriber(n, "/dso/odom");
	initSubscriber(n, "/bebop/cmd_vel");
        initPublisher(n, "/ultimetry");
    }

    void initPublisherAndSubscribers(ros::NodeHandle n, std::string droneOdometryTopicName, std::string dsoOdometryTopicName,
				    std::string droneCommandTopicName, std::string outputTopicName)
    {
	droneOdometrySubscriber.initSubscriber(n, droneOdometryTopicName);
	dsoOdometrySubscriber.initSubscriber(n, dsoOdometryTopicName);
	initSubscriber(n, droneCommandTopicName);
        initPublisher(n, outputTopicName);
    }

    Ultimetry() 
    { 
        newMessage = false;
	count = 0;
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    ultimetryPublisher.publish(odometry);
	    newMessage = false;
	}
    }

    private:
    ros::Publisher ultimetryPublisher;
    OdometrySubscriber droneOdometrySubscriber;
    OdometrySubscriber dsoOdometrySubscriber;
    ros::Subscriber droneCommandSubscriber;

    bool newMessage;
    int count;
    nav_msgs::Odometry odometry;

    void initSubscriber(ros::NodeHandle n, std::string droneCommandTopicName)
    {
        droneCommandSubscriber = n.subscribe(droneCommandTopicName, 50, &Ultimetry::droneCommandCallback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        ultimetryPublisher = n.advertise<nav_msgs::Odometry>(outputTopicName, 50); 
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ultimetry");
    ros::NodeHandle n;

    ROS_INFO("Have %i arguments passed.", argc);

    std::string outputTopicName;
    std::string droneOdometryTopicName;
    std::string dsoOdometryTopicName;
    std::string droneCommandTopicName;
    bool argumentsPassed = false;

    if (argc >= 3)
    {
    	outputTopicName = argv[1];
	droneOdometryTopicName = argv[2];
        dsoOdometryTopicName = "/dso/odom";
	droneCommandTopicName = "/bebop/cmd_vel";
	if (argc >= 4)
	{
	    dsoOdometryTopicName = argv[3];
	    if (argc == 5)
	    {
		droneCommandTopicName = argv[4];
	    }
	}
	argumentsPassed = true;
    } 

    Ultimetry ult;

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s, %s, %s", droneOdometryTopicName.c_str(), dsoOdometryTopicName.c_str(), 
					       droneCommandTopicName.c_str(), outputTopicName.c_str());
	ult.initPublisherAndSubscribers(n, droneOdometryTopicName, dsoOdometryTopicName, droneCommandTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /bebop/odom and /bebop/frodopathy.");
	ult.initPublisherAndSubscribers(n);
    }

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ult.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
