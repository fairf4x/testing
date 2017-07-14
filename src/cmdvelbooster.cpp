#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

class CmdVelBooster
{
    struct boostInfo
    {
	bool isBoosting;
	long boostEnds;
	double boostValue;
	double compensation;

	double last;
	double actual;

	std::string boostName;

	boostInfo(double comp, std::string name)
	{
	    isBoosting = false;
	    boostValue = 0;
	    boostEnds = -1;

	    last = 0;
	    actual = 0;

	    compensation = comp;
	    boostName = name;
	}
    };

    public:
    void resolveBoosting(geometry_msgs::Twist actualMessage)
    {
	boostX_.last = boostX_.actual;
	boostX_.actual = actualMessage.linear.x;

	boostY_.last = boostY_.actual;
	boostY_.actual = actualMessage.linear.y;

	resolveBoost(boostX_);
	resolveBoost(boostY_);
    }

    bool isBoostingX()
    {
	return boostX_.isBoosting;
    }

    bool isBoostingY()
    {
	return boostY_.isBoosting;
    }

    double getBoostX()
    {
	return boostX_.boostValue;
    }

    double getBoostY()
    {
	return boostY_.boostValue;
    }

    CmdVelBooster(double compX, double compY, double lim) 
	: boostX_(compX, "X"),
	  boostY_(compY, "Y")
    {
	limit = lim;
    }

    private:
    geometry_msgs::Twist lastMessage;
    double limit;

    boostInfo boostX_;
    boostInfo boostY_;

    void resolveBoost(boostInfo boost)
    {
	double diff = std::abs(boost.actual - boost.last);
	if (diff > limit * 0.8)
	{
	    if (!haveSameSign(boost.actual, boost.last))
	    {
		boost.isBoosting = true;
		boost.boostValue = boost.actual*(2 + 5*boost.compensation);
		boost.boostEnds = 20;
		ROS_INFO("Counter-boosting %s.", boost.boostName.c_str());
	    }

	    if (!boost.isBoosting)
	    {
		boost.isBoosting = true;
		boost.boostValue = boost.actual*(2 + 3*boost.compensation);
		boost.boostEnds = 10;
		ROS_INFO("Boosting %s.", boost.boostName.c_str());
	    }
	}

	if (boost.isBoosting && --boost.boostEnds < 0)
	{
	    boost.isBoosting = false;
	    boost.boostValue = 0;
	    boost.boostEnds = -1;
	    ROS_INFO("End of boosting %s.", boost.boostName.c_str());
	}
    }

    bool haveSameSign(double a, double b)
    {
	if ((a > 0 && b > 0) || (a < 0 && b < 0))
	    return true;

	return false;
    }
};
