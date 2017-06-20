#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testparam");

  double p1;
  double p2;

  ros::NodeHandle n;
  n.param<double>("testParam1", p1, 1.11);
  n.param<double>("testParam2", p2, 2.22);

  ROS_INFO("Got params %f and %f. Can count with them (p1*p2): %f!", p1, p2, p1*p2);

  return 0;
}
