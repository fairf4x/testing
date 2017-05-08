#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <string>

class OdometrySubscriber
{
    public:
    void callback(const nav_msgs::Odometry::ConstPtr& odom)
    {
	nav_msgs::Odometry newOdometry;
	newOdometry.header = odom->header;
	newOdometry.child_frame_id = odom->child_frame_id;
	newOdometry.pose = odom->pose;
	newOdometry.twist = odom->twist;
	
	count++;
	hasNew = true;

	odometry = newOdometry;
    }

    OdometrySubscriber()
    {
	hasNew = false;
	count = 0;
    }

    bool HasNewMessage()
    {
	return hasNew;
    }

    int GetCount()
    {
	return count;
    }

    nav_msgs::Odometry GetNewest()
    {
	hasNew = false;
	count = 0;
	return odometry;
    }

    void initSubscriber(ros::NodeHandle n, std::string odometryTopicName)
    {
        subscriber = n.subscribe(odometryTopicName, 50, &OdometrySubscriber::callback, this);
    }

    private:
    ros::Subscriber subscriber;
    nav_msgs::Odometry odometry;
    int count;
    bool hasNew;
};

class Ultimetry
{
    public:
    void droneCommandCallback(const geometry_msgs::Twist::ConstPtr& cmd)
    { 
	ROS_INFO("I heard command.");
	count++;
	newMessage = true;
	geometry_msgs::Twist newMessage;

	newMessage.linear = cmd->linear;
	newMessage.angular = cmd->angular;
	joyMessage = newMessage;
    }

    void initPublisherAndSubscribers(ros::NodeHandle n)
    {
	droneOdometrySubscriber.initSubscriber(n, "/bebop/odom");
	dsoOdometrySubscriber.initSubscriber(n, "/dso_odom_topic");
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
	messageTopicName = "/ultimetry/message";
    }

    void publishIfNew()
    {
	if (droneOdometrySubscriber.HasNewMessage())
	{
	    int droneOdomCount = droneOdometrySubscriber.GetCount();
	    int dsoOdomCount = dsoOdometrySubscriber.GetCount();

	    nav_msgs::Odometry droneOdom = droneOdometrySubscriber.GetNewest();
	    dsoOdometrySubscriber.GetNewest();

	    std::ostringstream stringStream;
	    stringStream << "Drone odometry has " << droneOdomCount << " messages, dso odometry has " << dsoOdomCount << " messages and  commands have " << count << " messages. \n Cmd: " << joyMessage.angular.z;
	    std_msgs::String message;
	    message.data = stringStream.str();

	    droneOdom.pose.pose.position.x = droneOdom.pose.pose.position.x + 0.3;
	    ultimetryPublisher.publish(droneOdom);
	    ultimetryMessagePublisher.publish(message);
	    newMessage = false;
	    count = 0;
	}
    }

    private:
    ros::Publisher ultimetryPublisher;
    ros::Publisher ultimetryMessagePublisher;
    OdometrySubscriber droneOdometrySubscriber;
    OdometrySubscriber dsoOdometrySubscriber;
    ros::Subscriber droneCommandSubscriber;
    geometry_msgs::Twist joyMessage;

    bool newMessage;
    int count;
    nav_msgs::Odometry odometry;
    std::string messageTopicName;

    void initSubscriber(ros::NodeHandle n, std::string droneCommandTopicName)
    {
        droneCommandSubscriber = n.subscribe(droneCommandTopicName, 50, &Ultimetry::droneCommandCallback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        ultimetryPublisher = n.advertise<nav_msgs::Odometry>(outputTopicName, 1);
	ultimetryMessagePublisher = n.advertise<std_msgs::String>(messageTopicName, 1);  
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
        dsoOdometryTopicName = "/dso_odom_topic";
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
