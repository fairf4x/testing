#include "ros/ros.h"
#include "testing/GetEigenValue.h"
#include "testing/ComputeTransformationMatrix.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>

class EigenWrapper
{
public:
  bool getEigenValue(testing::GetEigenValue::Request& req,
                     testing::GetEigenValue::Response& res)
  {
    Eigen::Matrix2f A;
    A << req.matrix[0], req.matrix[1], req.matrix[2], req.matrix[3];

    Eigen::EigenSolver<Eigen::Matrix2f> eigensolver(A);
    if (eigensolver.info() != Eigen::Success) abort();
    
    Eigen::Vector2cf result;
    result = eigensolver.eigenvalues();

    res.eigen1[0] = result.real()[0];
    res.eigen2[0] = result.real()[1];
   // res.eigen3[0] = result.real()[2];
    res.eigen1[1] = result.imag()[0];
    res.eigen2[1] = result.imag()[1];
   // res.eigen3[1] = result.imag()[2];

   ROS_INFO("request: \nA=%f %f \n  %f %f", req.matrix[0], req.matrix[1], req.matrix[2], req.matrix[3]);
	//	req.matrix[4], req.matrix[5], req.matrix[6], req.matrix[7], req.matrix[8]);
    ROS_INFO("sending back response: [%f %f] [%f %f] ", res.eigen1[0], res.eigen1[1],
			res.eigen2[0], res.eigen2[1]);

    return true;
  }

bool computeTransformationMatrix(testing::ComputeTransformationMatrix::Request& req,
                     		 testing::ComputeTransformationMatrix::Response& res)
  {
    int sizeOne = req.firstspacepoints.size();
    int sizeTwo = req.secondspacepoints.size();
    int totalSize = sizeOne;
    if (sizeTwo < totalSize)
    {
	totalSize = sizeTwo;
    }
    if (totalSize < 3)
    {
	ROS_ERROR("The size of the smaller point set was lesser then 3! Terminating the transformation matrix generation.");
	return true;
    }

    Eigen::Matrix<float,3,Eigen::Dynamic> spaceOne;
    Eigen::Matrix<float,3,Eigen::Dynamic> spaceTwo;
    spaceOne.resize(3,totalSize);
    spaceTwo.resize(3,totalSize);
    for(int i = 0; i < totalSize; i++)
    {
	Eigen::Vector3f vectorOne(req.firstspacepoints[i].x, req.firstspacepoints[i].y, req.firstspacepoints[i].z);
	spaceOne.col(i) = vectorOne;
	Eigen::Vector3f vectorTwo(req.secondspacepoints[i].x, req.secondspacepoints[i].y, req.secondspacepoints[i].z);
	spaceTwo.col(i) = vectorTwo;
    }

//	Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	std::cout << spaceOne << "\n";

    Eigen::Matrix<float,4,4> result;
    result = Eigen::umeyama(spaceOne,spaceTwo,false);

    res.transformation.translation.x = result(0,3);
    res.transformation.translation.y = result(1,3);
    res.transformation.translation.z = result(2,3);

    Eigen::Quaternionf q(result.block<3,3>(0,0));
    res.transformation.rotation.x = q.x();
    res.transformation.rotation.y = q.y();
    res.transformation.rotation.z = q.z();
    res.transformation.rotation.w = q.w();

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "eigen_wrapper_server");
  ros::NodeHandle n;

  EigenWrapper eigen;
  ros::ServiceServer ss = n.advertiseService("get_eigen_value", &EigenWrapper::getEigenValue, &eigen);
  ros::ServiceServer ss2 = n.advertiseService("compute_transformation_matrix", &EigenWrapper::computeTransformationMatrix, &eigen);

  ros::spin();

  return 0;
}
