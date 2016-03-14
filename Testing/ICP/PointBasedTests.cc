#include <catch.hpp>
#include <Eigen/Dense>
#include "PointCloud.h"
#include "ICPRegistration.h"
#include "utils.h"
#include "defs.h"
#include <iostream>


using namespace ICP_MPHYG02;

TEST_CASE("Test whether the PointBasedRegistration output is correct" , "[pointbased]")
{
    PointCloud pPCD(PATH_IN_FIXED_PTS, POINT_BASED_FLAG);
    PointCloud qPCD(PATH_IN_MOVING_PTS, POINT_BASED_FLAG);
    // PointCloud qPCD_GT(qPCD);

    ICPRegistration PointRegistration(POINT_BASED_FLAG);
    double RMS = 0;
    Eigen::Matrix4d finalTransf;
    PointRegistration.solve(pPCD, qPCD, RMS, finalTransf, PATH_OUTPUT_PTS);
    // Eigen::Matrix4d groundTruthTransf(Eigen::Matrix4d::Identity());
    // readTransfFromFile(groundTruthTransf, PATH_GT_TRANSF_PTS);
    // std::cout << "ground truth transf = \n" << groundTruthTransf << std::endl;
    // CAPTURE( RMS );
    REQUIRE( RMS < 0.00001 );
}
TEST_CASE("Test LS estimation" , "[pointbased_ls]")
{
    Eigen::MatrixXd pMat(Eigen::MatrixXd::Random(3,8));

    PointCloud pPCD(pMat);

    ICPRegistration PointRegistration(POINT_BASED_FLAG);
    double RMS = 0;
    Eigen::Matrix4d finalTransf;

    SECTION( "throws exception (LSestimation) - unequal point set sizes" ) {
        Eigen::MatrixXd qMatTemp(Eigen::MatrixXd::Random(3,9));
        PointCloud qPCDTemp(qMatTemp);

        REQUIRE_THROWS(PointRegistration.getLSEstimate(pPCD, qPCDTemp, finalTransf,RMS));
    }
}
