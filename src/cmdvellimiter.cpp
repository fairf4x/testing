#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

class CmdVelLimiter
{
    public:
    void callback(const geometry_msgs::Twist::ConstPtr& twist)
    { 
        newMessage = true;
	lastMessage.linear = twist->linear;
	lastMessage.angular = twist->angular;

	processLastMessage();
    }

    void initPublisherAndSubscriber(ros::NodeHandle n, std::string sourceTopicName, std::string outputTopicName)
    {
	initSubscriber(n, sourceTopicName);
        initPublisher(n, outputTopicName);
    }

    CmdVelLimiter() 
    {
	newMessage = false;
	limit = 0.5;
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    cmdVelPublisher.publish(lastMessage);
            newMessage = false;
	}
    }

    private:
    ros::Publisher cmdVelPublisher;
    ros::Subscriber cmdVelSubscriber;

    geometry_msgs::Twist lastMessage;
    bool newMessage;
    float limit;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        cmdVelSubscriber = n.subscribe(sourceTopicName, 50, &CmdVelLimiter::callback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        cmdVelPublisher = n.advertise<geometry_msgs::Twist>(outputTopicName, 50); 
    }

    void processLastMessage()
    {
	if (lastMessage.linear.x > limit)
	    lastMessage.linear.x = limit;

	if (lastMessage.linear.y > limit)
	    lastMessage.linear.y = limit;

	if (lastMessage.angular.z > limit)
	    lastMessage.angular.z = limit;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "cmdVelLimiter");
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

    CmdVelLimiter limiter;

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	limiter.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /cmd_vel_to_limit and /limiter/cmd_vel.");
	limiter.initPublisherAndSubscriber(n, "/cmd_vel_to_limit", "/limiter/cmd_vel");
    }

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        limiter.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
