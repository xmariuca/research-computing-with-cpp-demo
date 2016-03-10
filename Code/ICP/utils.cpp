#include "utils.h"
#include  <iomanip> // setprecision

namespace ICP_MPHYG02
{
    void printMessage(std::string text)
    {
        std::cout<< text << std::endl;
    }
    void writeToFile(Eigen::Matrix4d& matrix, std::string filename)
    {
        std::ofstream outStream;
        outStream.open(filename);
        if (outStream.is_open())
        {
            int32_t numRows = matrix.rows();
            int32_t numCols = matrix.cols();
            for (int32_t irow = 0; irow < numRows; irow++)
            {
                for (int32_t icol = 0; icol < numCols; icol++)
                {
                    outStream << std::setprecision(3) << matrix(irow,icol) << "\t";

                }
                outStream << std::endl;
            }
        }
        else
        {
            std::cout << "Couldn't open output file.\n";
        }
        outStream.close();
    }
}//namespace ICP_MPHYG02
