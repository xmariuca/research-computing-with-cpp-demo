#include <catch.hpp>
#include <string>
#include <Eigen/Dense>
#include "PointCloud.h"
#include "ICPRegistration.h"
#include "utils.h"
#include "defs.h"
#include <iostream>



static const char* PATH_IN_FIXED = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/fixed.txt";
static const char* PATH_IN_MOVING = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/moving.txt";
static std::string  PATH_GT_TRANSF = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/matrix4x4.txt";
static std::string PATH_OUTPUT = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/outputTest.txt";

using namespace ICP_MPHYG02;


TEST_CASE("Test whether the PointBasedRegistration output is correct" , "[pointbased]")
{
    PointCloud pPCD(PATH_IN_FIXED, POINT_BASED_FLAG);
    PointCloud qPCD(PATH_IN_MOVING, POINT_BASED_FLAG);
    // PointCloud qPCD_GT(qPCD);

    ICPRegistration PointRegistration(POINT_BASED_FLAG);
    double RMS = 0;
    Eigen::Matrix4d finalTransf;
    PointRegistration.solve(pPCD, qPCD, RMS, finalTransf, PATH_OUTPUT);
    // Eigen::Matrix4d groundTruthTransf(Eigen::Matrix4d::Identity());
    // readTransfFromFile(groundTruthTransf, PATH_GT_TRANSF);
    // std::cout << "ground truth transf = \n" << groundTruthTransf << std::endl;
    // CAPTURE( RMS );
    REQUIRE( RMS < 0.00001 );
}
