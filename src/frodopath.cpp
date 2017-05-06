#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <string>

class Frodopathy
{
    public:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    { 
	ROS_INFO("I heard: [%s]", odom->header.frame_id.c_str());

	path.header.seq = ++count;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "odom";

	geometry_msgs::PoseStamped poses;
	poses.header = odom->header;
	poses.pose = odom->pose.pose;

	path.poses.push_back(poses);

	newMessage = true;
    }

    void initPublisherAndSubscriber(ros::NodeHandle n)
    {
	initSubscriber(n, "/bebop/odom");
        initPublisher(n, "/bebop/frodopathy");
    }

    void initPublisherAndSubscriber(ros::NodeHandle n, std::string sourceTopicName, std::string outputTopicName)
    {
	initSubscriber(n, sourceTopicName);
        initPublisher(n, outputTopicName);
    }

    Frodopathy() 
    { 
        newMessage = false;
	count = 0;
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    frodopathyPublisher.publish(path);
	    newMessage = false;
	}
    }

    private:
    ros::Publisher frodopathyPublisher;
    ros::Subscriber odometrySubscriber;

    nav_msgs::Path path;
    bool newMessage;
    int count;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        odometrySubscriber = n.subscribe(sourceTopicName, 50, &Frodopathy::odomCallback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        frodopathyPublisher = n.advertise<nav_msgs::Path>(outputTopicName, 50); 
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "frodopath");
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

    Frodopathy frodo;

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	frodo.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /bebop/odom and /bebop/frodopathy.");
	frodo.initPublisherAndSubscriber(n);
    }

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        frodo.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
