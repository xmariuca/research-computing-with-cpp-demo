#include "PointCloud.h"
#include "utils.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>



#define FILE_1Point1Line 0
#define FILE_3Points1Line 1

namespace ICP_MPHYG02
{
    // PointCloud::PointCloud()
    // {
    //     m_pointSet = Matrix3d::Zero();
    //     m_pointsNumber = 0;
    // }
    // ************************************************************************
    PointCloud::PointCloud(Eigen::MatrixXd& in_pSet)
    {
        // check that it is in the format of 3 x numPts
        m_pointSet = in_pSet;
        m_pointsNumber = m_pointSet.cols();
        printMessage("* Point cloud object created!");
    }
    PointCloud::PointCloud(std::string filename, int flag)
    {
        // #define FILE_1Point1Line 0
        if(flag == FILE_1Point1Line)
        {
            std::ifstream inputStream;
            inputStream.open(filename);
            std::vector<std::string> listLines;
            if (inputStream.is_open()) {
                while (!inputStream.eof()) {
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
            // save points in my member variables
            m_pointsNumber = listLines.size();
            if (m_pointsNumber)
            {
                m_pointSet = Eigen::MatrixXd::Zero(3,m_pointsNumber);
                for (int i = 0; i <= m_pointsNumber; i++)
                {
                    int idxCoord = 0;
                    std::stringstream stream(listLines[i]);
                    float coord;
                    while(stream >> coord)
                    {
                        if(!stream)
                            break;
                        m_pointSet(idxCoord,i) = coord;
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
    }
    PointCloud::~PointCloud()
    {
    }
    PointCloud::PointCloud(const PointCloud& in_pSrc)
    {
        m_pointSet = in_pSrc.m_pointSet;
        m_pointsNumber = in_pSrc.m_pointsNumber;

    }
    PointCloud& PointCloud::operator= (const PointCloud &in_pSrc)
    {
        // check for self - assignment
        if (this == &in_pSrc)
        return *this;

        m_pointSet = in_pSrc.m_pointSet;
        m_pointsNumber = in_pSrc.m_pointsNumber;

        // return the existing object
        return *this;
    }
    // ************************************************************************
    void PointCloud::getCenteredPCD(Eigen::MatrixXd& out_centeredPCD)
    {
        Eigen::Vector3d meanPCD = Eigen::Vector3d::Zero();
        PointCloud::getMeanPCD(meanPCD);
        // std::cout << "* Mean PCD = " << meanPCD << std::endl;
        out_centeredPCD = m_pointSet.colwise() - meanPCD;
    }
    // ************************************************************************
    void PointCloud::getMeanPCD(Eigen::Vector3d& out_mean)
    {
        Eigen::Vector3d meanP(m_pointSet.row(0).mean(),
        m_pointSet.row(1).mean(),
        m_pointSet.row(2).mean());
        out_mean = meanP;
        // std::cout << "* Mean PCD = " << out_mean << std::endl;
    }
    // ************************************************************************
    void PointCloud::applyTransformation(Eigen::Matrix4d& transfMat)
    {
        std::cout << m_pointSet << std::endl;

        for ( int i = 0; i < m_pointsNumber; i++)
        {
            Eigen::Vector4d thisCoord = m_pointSet.col(i).homogeneous();
            Eigen::Vector4d newCoord = transfMat * thisCoord;
            m_pointSet.col(i) = newCoord.block(0,0,3,1);
        }
        std::cout << m_pointSet << std::endl;
    }

}    //namespace ICP_MPHYG02
