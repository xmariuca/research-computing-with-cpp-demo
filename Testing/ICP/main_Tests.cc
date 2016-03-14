#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>
#include <Eigen/Dense>
#include "defs.h"
#include "PointCloud.h"
using namespace ICP_MPHYG02;

TEST_CASE("Test whether the PointCloud class has correct methods", "[pointcloudclass]")
{
    Eigen::MatrixXd mat(Eigen::MatrixXd::Random(3,5));
    PointCloud pcd(mat);

    REQUIRE(mat.rows() == 3);
    REQUIRE(mat.cols() == 5);
    REQUIRE(pcd.getPointsNum() == mat.cols());

    Eigen::Vector3d meanVect = mat.rowwise().mean();

    SECTION( "computes mean correctly" ) {
        Eigen::Vector3d outMean;
        pcd.getMeanPCD(outMean);

        REQUIRE((meanVect - outMean).norm() < 0.0001 );
    }

    SECTION( "centers point set correctly" ) {
        Eigen::MatrixXd centeredMat = mat.colwise() - meanVect;

        Eigen::MatrixXd out_centeredPCD;
        pcd.getCenteredPCD(out_centeredPCD);
        Eigen::VectorXd RMSVect = (out_centeredPCD - centeredMat).colwise().norm();
        REQUIRE(RMSVect.sum() < 0.0001 );
    }

}
TEST_CASE("Test whether the PointCloud throws exceptions - negative tests", "[pointcloudclass_negative]")
{
    Eigen::MatrixXd mat(Eigen::MatrixXd::Zero(3,8));

    SECTION( "throws exception - empty point set" ) {
        REQUIRE_THROWS(PointCloud pcd(mat));
    }
    SECTION( "throws exception - incorrect file format PointBased" ) {
        REQUIRE_THROWS(PointCloud pcd(PATH_IN_FIXED_SURF_SMALL,POINT_BASED_FLAG));
    }
    SECTION( "throws exception - incorrect file format SurfaceBased" ) {
        REQUIRE_THROWS(PointCloud pcd(PATH_IN_FIXED_PTS,SURFACE_BASED_FLAG));
    }
    SECTION( "throws exception - empty file" ) {
        REQUIRE_THROWS(PointCloud pcd(PATH_EMPTY_PTS,POINT_BASED_FLAG));
    }
    SECTION( "throws exception - invalid transformation" ) {
        Eigen::Matrix4d transfMat(Eigen::Matrix4d::Random());
        transfMat(3,3) = 5;
        mat = Eigen::MatrixXd::Random(3,8);
        PointCloud pcd(mat);
        REQUIRE_THROWS(pcd.applyTransformation(transfMat));
    }
    SECTION( "throws exception - incorrect size" ) {
        mat.resize(4,6);
        REQUIRE(mat.rows() == 4);
        REQUIRE_THROWS(PointCloud pcd(mat));
    }

}
