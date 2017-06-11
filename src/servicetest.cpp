#include "ros/ros.h"
#include "testing/GetEigenValue.h"
#include "testing/ComputeTransformationMatrix.h"
#include "geometry_msgs/Point.h"
#include "Eigen/Core"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eigen_wrapper_test");

  ros::NodeHandle n;
  ros::ServiceClient eigenClient = n.serviceClient<testing::GetEigenValue>("get_eigen_value");
  ros::ServiceClient transformationClient = n.serviceClient<testing::ComputeTransformationMatrix>("compute_transformation_matrix");
  testing::GetEigenValue eigen;
  testing::ComputeTransformationMatrix transformation;

  geometry_msgs::Point firstPoint;
  geometry_msgs::Point secondPoint;
  geometry_msgs::Point thirdPoint;
  geometry_msgs::Point fourthPoint;
  firstPoint.x = 1;
  firstPoint.y = 0;
  firstPoint.z = 0;
  secondPoint.x = 0;
  secondPoint.y = 1;
  secondPoint.z = 0;
  thirdPoint.x = 0;
  thirdPoint.y = 0;
  thirdPoint.z = 1;
  fourthPoint.x = 0;
  fourthPoint.y = 1;
  fourthPoint.z = 1;
  transformation.request.firstspacepoints.push_back(firstPoint);
  transformation.request.firstspacepoints.push_back(secondPoint);
  transformation.request.firstspacepoints.push_back(thirdPoint);
  transformation.request.firstspacepoints.push_back(fourthPoint);

  geometry_msgs::Point fPoint;
  geometry_msgs::Point sPoint;
  geometry_msgs::Point tPoint;
  geometry_msgs::Point foPoint;
  fPoint.x = 0;
  fPoint.y = 2;
  fPoint.z = 0;
  sPoint.x = 0;
  sPoint.y = 0;
  sPoint.z = 2;
  tPoint.x = 2;
  tPoint.y = 0;
  tPoint.z = 0;
  foPoint.x = 2.1;
  foPoint.y = 0;
  foPoint.z = 2;
  transformation.request.secondspacepoints.push_back(fPoint);
  transformation.request.secondspacepoints.push_back(sPoint);
  transformation.request.secondspacepoints.push_back(tPoint);
  transformation.request.secondspacepoints.push_back(foPoint);

   ROS_INFO("Calling service.");

    if (transformationClient.call(transformation))
    {
//       ROS_INFO("Received response: [%f %f] [%f %f] ", srv.response.eigen1[0], srv.response.eigen1[1],
//			srv.response.eigen2[0], srv.response.eigen2[1]);
   ROS_INFO("Received response: \nt= %f %f %f \nq= %f %f %f %f",
		transformation.response.transformation.translation.x,
		transformation.response.transformation.translation.y,
		transformation.response.transformation.translation.z,
		transformation.response.transformation.rotation.x,
		transformation.response.transformation.rotation.y,
		transformation.response.transformation.rotation.z,
		transformation.response.transformation.rotation.w);
    }
    else
    {
      ROS_ERROR("Failed to call service transformation.");
      return 1;
    }


  return 0;
}
