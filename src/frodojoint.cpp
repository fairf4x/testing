#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include <string>

class Frodopathy
{
    public:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
    { 
	ROS_INFO("I heard: [%s]", odom->header.frame_id.c_str());

	joint.header.seq = ++count;
	joint.header.stamp = ros::Time::now();
	joint.header.frame_id = "odom";

	joint.name.resize(1);
	joint.position.resize(1);
	joint.velocity.resize(1);

	joint.name[0] = jointName;
	joint.position[0] = ;
	joint.velocity[0] = ;

	newMessage = true;
    }

    void initPublisherAndSubscriber(ros::NodeHandle n, std::string sourceTopicName, std::string outputTopicName)
    {
	initSubscriber(n, sourceTopicName);
        initPublisher(n, outputTopicName);
    }

    void setJointName(std::string jointName)
    {
	this.jointName = jointName;
    }

    Frodojoint() 
    { 
        newMessage = false;
	count = 0;
	frodojoint = "Base";
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    frodojointPublisher.publish(joint);
	    newMessage = false;
	}
    }

    private:
    ros::Publisher frodojointPublisher;
    ros::Subscriber odometrySubscriber;

    sensor_msgs::JointState joint;
    bool newMessage;
    int count;
    std::string jointName;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        odometrySubscriber = n.subscribe(sourceTopicName, 50, &Frodojoint::odomCallback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        frodojointPublisher = n.advertise<sensor_msgs::JointState>(outputTopicName, 50); 
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "frodojoint");
    ros::NodeHandle n;

    ROS_INFO("Have %i arguments passed.", argc);

    std::string sourceTopicName;
    std::string outputTopicName;
    std::string jointName;
    bool argumentsPassed = false;

    if (argc == 4)
    {
    	sourceTopicName = argv[1];
	outputTopicName = argv[2];
	jointName = argv[3];
	argumentsPassed = true;
    } 

    Frodojoint frodo;

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	frodo.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
	frodo.setJointName(jointName);
    }
    else
    {
        ROS_INFO("Got no params, using /bebop/odom and /bebop/frodojoint.");
	frodo.initPublisherAndSubscriber(n, "/bebop/odom", "/bebop/frodojoint");
    }

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        frodo.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
