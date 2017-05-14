#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>

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
	
	hasNew = true;
	odometry = newOdometry;
    }

    OdometrySubscriber()
    {
	hasNew = false;
    }

    bool HasNewMessage()
    {
	return hasNew;
    }

    nav_msgs::Odometry GetNewest()
    {
	hasNew = false;
	return odometry;
    }

    void initSubscriber(ros::NodeHandle n, std::string odometryTopicName)
    {
        subscriber = n.subscribe(odometryTopicName, 50, &OdometrySubscriber::callback, this);
    }

    private:
    ros::Subscriber subscriber;
    nav_msgs::Odometry odometry;
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

	resolveRotation(std::abs(cmd->angular.z));
    }

    void resolveRotation(float valueZ)
    {
	float valueLimit = 0.1;
	int consecutiveLimit = 3;
	
	if (rotatingCommandActive)
	{
	    if (valueZ < valueLimit)
	    {
		if(++rotationCommandCount == consecutiveLimit)
		{
		    rotatingCommandActive = false;
		    rotationCommandCount = 0;
		}
	    }
	    else
	    {
		rotationCommandCount = 0;
	    }
	}
	else
	{
	    if (valueZ > valueLimit)
	    {
		if(++rotationCommandCount == consecutiveLimit)
		{
		    rotatingCommandActive = true;
		    rotationCommandCount = 0;
		}
	    }
	    else
	    {
		rotationCommandCount = 0;
	    }
	}
	
	lastAngularZ = valueZ;
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

	rotatingCommandActive = false;
	rotationCommandCount = 0;
	lastAngularZ = 0;

	dsoSumSinceLastTime.linear.x = 0;
	dsoSumSinceLastTime.linear.y = 0;
        dsoSumSinceLastTime.linear.z = 0;

        droneCumulative.linear.x = 0;
	droneCumulative.linear.y = 0;
	droneCumulative.linear.z = 0;

	droneLastPosition.position.x = 0;
	droneLastPosition.position.y = 0;
	droneLastPosition.position.z = 0;
    }

    void publishIfNew()
    {
	std_msgs::String message;
	if (dsoOdometrySubscriber.HasNewMessage())
	{
	    std::ostringstream stringStream;
    	    nav_msgs::Odometry dsoOdom = dsoOdometrySubscriber.GetNewest();

	    dsoSumSinceLastTime.linear.x += dsoOdom.twist.twist.linear.x;
	    dsoSumSinceLastTime.linear.y += dsoOdom.twist.twist.linear.y;
	    dsoSumSinceLastTime.linear.z += dsoOdom.twist.twist.linear.z;

	    message.data = stringStream.str();
            ultimetryMessagePublisher.publish(message);
	}

	if (droneOdometrySubscriber.HasNewMessage())
	{
	    nav_msgs::Odometry droneOdom = droneOdometrySubscriber.GetNewest();
	    
	    if(!rotatingCommandActive && !droneLastPositionStarting())
	    {
		geometry_msgs::Twist dronePositionDifference;
		dronePositionDifference.linear.x = droneOdom.pose.pose.position.x - droneLastPosition.position.x - droneCumulative.linear.x;
		dronePositionDifference.linear.y = droneOdom.pose.pose.position.y - droneLastPosition.position.y - droneCumulative.linear.y;
		dronePositionDifference.linear.z = droneOdom.pose.pose.position.z - droneLastPosition.position.z - droneCumulative.linear.z;
		
		double length = countVectorLength(dronePositionDifference.linear);
		if (length > 0.1)
		{
		    droneCumulative.linear.x += dronePositionDifference.linear.x * 0.1;
		    droneCumulative.linear.y += dronePositionDifference.linear.y * 0.1;
		    droneCumulative.linear.z += dronePositionDifference.linear.z * 0.1;
		}
		else if (length > 0.08)
		{
		    droneCumulative.linear.x += dronePositionDifference.linear.x * 0.05;
		    droneCumulative.linear.y += dronePositionDifference.linear.y * 0.05;
		    droneCumulative.linear.z += dronePositionDifference.linear.z * 0.05;
		}
/*		std::ostringstream stringStream;
		stringStream << "Dso: " << dsoSumSinceLastTime.linear.x << " " << dsoSumSinceLastTime.linear.y << " " << dsoSumSinceLastTime.linear.z 			<< "\nDso vector length: " << countVectorLength(dsoSumSinceLastTime.linear) 
		<< "\nDrone: " << droneOdom.twist.twist.linear.x << " " << droneOdom.twist.twist.linear.y << " " << droneOdom.twist.twist.linear.z    			<< "\nDrone difference: " << droneOdom.twist.twist.linear.x - droneLastTime.linear.x << " " 
<< droneOdom.twist.twist.linear.y - droneLastTime.linear.y << " " << droneOdom.twist.twist.linear.z - droneLastTime.linear.z 
		<< "\nDrone position: " << droneOdom.pose.pose.position.x << " " << droneOdom.pose.pose.position.y << " " << droneOdom.pose.pose.position.z
		<< "\nDrone position difference: " << dronePositionDifference.linear.x << " " << dronePositionDifference.linear.y << " " << dronePositionDifference.linear.z
		<< "\nDrone vector length: " << countVectorLength(dronePositionDifference.linear);
		message.data = stringStream.str();

	        ultimetryMessagePublisher.publish(message); */
	    }

	    droneOdom.pose.pose.position.x = droneOdom.pose.pose.position.x - droneCumulative.linear.x; 
	    droneOdom.pose.pose.position.y = droneOdom.pose.pose.position.y - droneCumulative.linear.y;
	    droneOdom.pose.pose.position.z = droneOdom.pose.pose.position.z - droneCumulative.linear.z;

	    ultimetryPublisher.publish(droneOdom);

	    newMessage = false;
	    count = 0;

	    dsoSumSinceLastTime.linear.x = 0;
	    dsoSumSinceLastTime.linear.y = 0;
	    dsoSumSinceLastTime.linear.z = 0;

//	    droneLastTime.linear = droneOdom.twist.twist.linear;
	    droneLastPosition.position = droneOdom.pose.pose.position;
	}
    }

    private:
    ros::Publisher ultimetryPublisher;
    ros::Publisher ultimetryMessagePublisher;
    OdometrySubscriber droneOdometrySubscriber;
    OdometrySubscriber dsoOdometrySubscriber;
    ros::Subscriber droneCommandSubscriber;
    geometry_msgs::Twist joyMessage;
    geometry_msgs::Twist dsoSumSinceLastTime;    
    geometry_msgs::Twist droneCumulative;
    geometry_msgs::Pose droneLastPosition;

    bool newMessage;
    int count;
    nav_msgs::Odometry odometry;
    std::string messageTopicName;

    bool rotatingCommandActive;
    int rotationCommandCount;
    float lastAngularZ;

    bool droneLastPositionStarting()
    {
	if (droneLastPosition.position.x != 0)
	    return false;
	
	if (droneLastPosition.position.y != 0)
	    return false;

	if (droneLastPosition.position.z != 0)
	    return false;	

	return true;
    }

    double countVectorLength(geometry_msgs::Vector3 v)
    {
	double base = v.x*v.x + v.y*v.y + v.z*v.z;
	return sqrt(base);
    }

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
