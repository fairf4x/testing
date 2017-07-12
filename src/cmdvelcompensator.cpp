#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

class CmdVelCompensator
{
    public:
    void callback(const geometry_msgs::Twist::ConstPtr& twist)
    { 
        newMessage = true;
	lastMessage = actualMessage;
	actualMessage.linear = twist->linear;
	actualMessage.angular = twist->angular;
    }

    void initPublisherAndSubscriber(ros::NodeHandle n, std::string sourceTopicName, std::string outputTopicName)
    {
	initSubscriber(n, sourceTopicName);
        initPublisher(n, outputTopicName);
    }

    CmdVelCompensator(double compX, double compY, double compZ, double compRot, double lim) 
    {
	newMessage = false;
	counter = 0;
	compensationX = compX;
	compensationY = compY;
	compensationZ = compZ;
	compensationRot = compRot;

	limit = lim;
        boostX = false;
	boostY = false;
	boostXValue = 0;
	boostYValue = 0;
	boostXEnd = -1;
	boostYEnd = -1;
    }

    void publishIfNew()
    {
	if (newMessage)
	{
	    counter++;
	    resolveBoost();
	    if (counter > 5)
	    {
		compensateActualMessage();
		if (counter > 6)
		{
		    counter = 0;
		}
	    }

	    cmdVelPublisher.publish(actualMessage);
            newMessage = false;
	}
    }

    private:
    ros::Publisher cmdVelPublisher;
    ros::Subscriber cmdVelSubscriber;

    int counter;

    geometry_msgs::Twist actualMessage;
    geometry_msgs::Twist lastMessage;
    bool newMessage;
    double compensationX;
    double compensationY;
    double compensationZ;
    double compensationRot;

    double limit;
    bool boostX;
    bool boostY;
    double boostXValue;
    double boostYValue;
    int boostXEnd;
    int boostYEnd;

    void initSubscriber(ros::NodeHandle n, std::string sourceTopicName)
    {
        cmdVelSubscriber = n.subscribe(sourceTopicName, 50, &CmdVelCompensator::callback, this);
    }

    void initPublisher(ros::NodeHandle n, std::string outputTopicName)
    {
        cmdVelPublisher = n.advertise<geometry_msgs::Twist>(outputTopicName, 50); 
    }

    void compensateActualMessage()
    {
	if (!boostX)
	{
	    actualMessage.linear.x += compensationX;
	}

	if (!boostY)
	{
	    actualMessage.linear.y += compensationY;
	}
    }

    void resolveBoost()
    {
	if (!boostX && std::abs(lastMessage.linear.x) != limit*2)
	{
	    double diff = std::abs(actualMessage.linear.x - lastMessage.linear.x);
	    if (diff > limit * 0.8)
	    {
		if (lastMessage.linear.x < actualMessage.linear.x)
		{
		    boostXValue = limit*2;
		}
		else
		{
		    boostXValue = -limit*2;
		}
		boostX = true;
		ROS_INFO("Boosting X.");

		if (diff < limit * 1.2)
		{
		    boostXEnd = 8;
		}
		else
		{
		    if (haveSameSign(actualMessage.linear.x, compensationX))
		    {
			boostXValue *= 1.35;
		    }
		    else
		    {
			double maxCoef = limit;
			if ((std::abs(compensationX)*5) < limit)
			{
			    maxCoef = std::abs(compensationX) * 5;
			}
			boostXValue *= (1 - maxCoef);
		    }
		    boostXEnd = 20;
		}
	    }
	}

	if (!boostY && std::abs(lastMessage.linear.y) != limit*2)
	{
	    double diff = std::abs(actualMessage.linear.y - lastMessage.linear.y);
	    if (diff > limit * 0.8)
	    {
		if (lastMessage.linear.y < actualMessage.linear.y)
		{
		    boostYValue = limit*2;
		}
		else
		{
		    boostYValue = -limit*2;
		}
		boostY = true;
		ROS_INFO("Boosting Y.");

		if (diff < limit * 1.2)
		{
		    boostYEnd = 8;
		}
		else
		{
		    if (haveSameSign(actualMessage.linear.y, compensationY))
		    {
			boostYValue *= 1.35;
		    }
		    else
		    {
			double maxCoef = limit;
			if ((std::abs(compensationY)*5) < limit)
			{
			    maxCoef = std::abs(compensationY) * 5;
			}
			boostYValue *= (1 - maxCoef);
		    }
		    boostYEnd = 20;
		}
	    }
	}

	if (boostX && --boostXEnd < 0)
	{
	    boostX = false;
	    boostXValue = 0;
	    boostXEnd = -1;
	    ROS_INFO("End of boosting X.");
	}

	if (boostY && --boostYEnd < 0)
	{
	    boostY = false;
	    boostYValue = 0;
	    boostYEnd = -1;
	    ROS_INFO("End of boosting Y.");
	}

	if (boostX)
	{
	    actualMessage.linear.x = boostXValue;
	}

	if (boostY)
	{
	    actualMessage.linear.y = boostYValue;
	}
    }

    bool haveSameSign(double a, double b)
    {
	if ((a > 0 && b > 0) || (a < 0 && b < 0))
	    return true;

	return false;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "cmdVelCompensator");
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

    double compensationX;
    double compensationY;
    double compensationZ;
    double compensationRot;
    n.param<double>("proclivity_x", compensationX, 0.0);
    n.param<double>("proclivity_y", compensationY, 0.0);
    n.param<double>("proclivity_z", compensationZ, 0.0);
    n.param<double>("proclivity_rot", compensationRot, 0.0);

    double limit;
    n.param<double>("cmd_vel_limit", limit, 0.3);

    CmdVelCompensator compensator(compensationX, compensationY, compensationZ, compensationRot, limit);

    if(argumentsPassed)
    {
        ROS_INFO("Got params: %s, %s", sourceTopicName.c_str(), outputTopicName.c_str());
	compensator.initPublisherAndSubscriber(n, sourceTopicName, outputTopicName);
    }
    else
    {
        ROS_INFO("Got no params, using /cmd_vel_to_compensate and /compensator/cmd_vel.");
	compensator.initPublisherAndSubscriber(n, "/cmd_vel_to_compensate", "/compensator/cmd_vel");
    }

    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        compensator.publishIfNew();
        ros::spinOnce();
        loop_rate.sleep();    
    }

    return 0;
}
