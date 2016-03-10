// #include <cstdlib>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "PointCloud.h"
#include "ICPRegistration.h"
#include "utils.h"

#define FILE_1Point1Line 0
#define FILE_3Points1Line 1

static const char* PATH_IN_FIXED = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/fixed.txt";
static const char* PATH_IN_MOVING = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/moving.txt";


using namespace ICP_MPHYG02;


int main(int argc, char** argv)
{
    printMessage("***");
    // Eigen::MatrixXd mat(3,5);
    // mat = 15*Eigen::MatrixXd::Random(3,5);
    //
    // PointCloud pFixed(mat);
    // std::cout << "numPts = " << pFixed.getPointsNum() << std::endl;
    //
    // Eigen::Matrix4d transfMat = Eigen::Matrix4d::Identity();
    // transfMat(0,3) = 15;
    // pFixed.applyTransformation(transfMat);

    PointCloud pPCD(PATH_IN_FIXED, FILE_1Point1Line);
    PointCloud qPCD(PATH_IN_MOVING, FILE_1Point1Line);

    ICPRegistration RegistrationSolver(pPCD, qPCD);
    Eigen::Matrix4d transfMatrix(Eigen::Matrix4d::Zero());
    double rmsError = 0;
    printMessage("Estimating transformation...");

    RegistrationSolver.getLSEstimate(transfMatrix, rmsError);
    // std::cout<< "Transf Matrix: \n" << transfMatrix << std::endl;
    std::cout<< "RMS error: " << rmsError << std::endl;


    printMessage("Magic!");
    return 1;
}
