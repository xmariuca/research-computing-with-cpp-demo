#ifndef _H_UTILS
#define _H_UTILS

#include <iostream>
#include <Eigen/Dense>
#include <fstream>


namespace ICP_MPHYG02
{
    void printMessage(std::string text);
    void writeToFile(Eigen::Matrix4d& matrix, std::string filename);
    void readTransfFromFile(Eigen::Matrix4d& out_matrix, std::string filename);

}//namespace ICP_MPHYG02
#endif
