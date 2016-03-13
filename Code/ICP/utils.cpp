#include "utils.h"
#include  <iomanip> // setprecision
#include <sstream>
#include <vector>
#include <string>

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
    void readTransfFromFile(Eigen::Matrix4d& out_matrix, std::string filename)
    {
        out_matrix = Eigen::Matrix4d::Identity();
        std::ifstream inputStream;
        inputStream.open(filename);
        std::vector<std::string> listLines;

        if (inputStream.is_open())
        {
            while (!inputStream.eof())
            {
                std::string tempLine;
                getline(inputStream,tempLine);
                listLines.push_back(tempLine);
                // std::cout << tempLine << "\n";
            }
        }
        else
        {
            std::cout << "Couldn't read file." << "\n";
        }
        inputStream.close();

        if (listLines.size() > 0)
        {
            for (int i = 0; i < listLines.size(); i++)
            {
                int idxCoord = 0;
                std::stringstream stream(listLines[i]);
                float coord;
                while(stream >> coord)
                {
                    if(!stream)
                    break;
                    out_matrix(i,idxCoord) = coord;
                    // std::cout << "Found coord: " << m_pointSet(idxCoord,i) << "\n";
                    idxCoord ++;
                }
            }
        }
        else
        {
            std::cout << "Empty file." << "\n";
        }

    }
}//namespace ICP_MPHYG02
