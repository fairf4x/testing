#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

class CmdVelBooster
{
    public:

    void resolveBoosting(geometry_msgs::Twist actualMessage)
    {
	resolveBoost();

	lastMessage = actualMessage;
    }

    bool isBoostingX()
    {
	return boostX;
    }

    bool isBoostingY()
    {
	return boostY;
    }

    double getBoostX()
    {
	return xBoostValue;
    }

    double getBoostY()
    {
	return yBoostValue;
    }

    CmdVelBooster(double compX, double compY, double compZ, double compRot, double lim) 
    {
	compensationX = compX;
	compensationY = compY;
	compensationZ = compZ;
	compensationRot = compRot;

	limit = lim;

        boostX = false;
	boostY = false;
	xBoostValue = 0;
	yBoostValue = 0;
	boostXEnd = -1;
	boostYEnd = -1;
    }

    private:
    geometry_msgs::Twist lastMessage;

    bool newMessage;
    double compensationX;
    double compensationY;
    double compensationZ;
    double compensationRot;

    double limit;

    bool boostX;
    bool boostY;
    double xBoostValue;
    double yBoostValue;
    int boostXEnd;
    int boostYEnd;

    void resolveBoost()
    {
	/*if (!boostX && std::abs(lastMessage.linear.x) != limit*2)
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
	}*/
    }

    bool haveSameSign(double a, double b)
    {
	if ((a > 0 && b > 0) || (a < 0 && b < 0))
	    return true;

	return false;
    }
};
