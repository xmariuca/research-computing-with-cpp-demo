#include <catch.hpp>
#include <string>
#include <Eigen/Dense>
#include "PointCloud.h"
#include "ICPRegistration.h"
#include "utils.h"
#include "defs.h"
#include <iostream>

using namespace ICP_MPHYG02;


TEST_CASE("Test whether the SurfaceBasedRegistration output is correct" , "[surfacebased]")
{
    PointCloud pPCD(PATH_IN_FIXED_SURF, SURFACE_BASED_FLAG);
    PointCloud qPCD(PATH_IN_MOVING_SURF, SURFACE_BASED_FLAG);

    ICPRegistration SurfaceRegistration(SURFACE_BASED_FLAG);
    double RMS = 0;
    Eigen::Matrix4d finalTransf;
    // std::string outputPath("SBF_Transf.txt");
    SurfaceRegistration.solve(pPCD, qPCD, RMS, finalTransf, PATH_OUTPUT_SURF);

    REQUIRE( RMS < 0.1 );
}
