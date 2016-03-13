#include <catch.hpp>
#include <string>
#include <Eigen/Dense>
#include "PointCloud.h"
#include "ICPRegistration.h"
#include "utils.h"
#include "defs.h"
#include <iostream>



static const char* PATH_IN_FIXED = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/fran_cut.txt";
static const char* PATH_IN_MOVING = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/fran_cut_transformed.txt";
static std::string  PATH_GT_TRANSF = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/matrix4x4.txt";
static std::string PATH_OUTPUT = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/outputTest.txt";

using namespace ICP_MPHYG02;


TEST_CASE("Test whether the SurfaceBasedRegistration output is correct" , "[surfacebased]")
{
    PointCloud pPCD(PATH_IN_FIXED, SURFACE_BASED_FLAG);
    PointCloud qPCD(PATH_IN_MOVING, SURFACE_BASED_FLAG);

    ICPRegistration SurfaceRegistration(SURFACE_BASED_FLAG);
    double RMS = 0;
    Eigen::Matrix4d finalTransf;
    // std::string outputPath("SBF_Transf.txt");
    SurfaceRegistration.solve(pPCD, qPCD, RMS, finalTransf, PATH_OUTPUT);

    REQUIRE( RMS < 0.1 );
}
