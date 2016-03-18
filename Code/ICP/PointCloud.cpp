#include "PointCloud.h"
#include "utils.h"
#include "defs.h"
#include "ExceptionIcp.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace ICP_MPHYG02
{
    // ************************************************************************
    PointCloud::PointCloud(Eigen::MatrixXd& in_pSet)
    {
        if(in_pSet.rows() != 3)
        {
            throw ExceptionIcp("The input point set has to be in the format 3xn, where n = number of points", "PointCloud.cpp");
        }
        if(in_pSet.isZero())
        {
            throw ExceptionIcp("Constructor called with empty point set", "PointCloud.cpp");
        }
        m_pointSet = in_pSet;
        m_pointsNumber = m_pointSet.cols();
        printMessage("* Point cloud object created!");
    }
    // ************************************************************************
    PointCloud::PointCloud(std::string filename, bool flag)
    {
        std::ifstream inputStream;
        inputStream.open(filename);
        std::vector<std::string> listLines;
        if (inputStream.is_open())
        {
            while (!inputStream.eof())
            {
                std::string tempLine;
                getline(inputStream,tempLine);
                if (tempLine.empty())
                {
                    std::cout << "Skipping empty line..." << std::endl;
                }
                else
                {
                    listLines.push_back(tempLine);
                }
            }
        }
        else
        {
            throw ExceptionIcp("Couldn't open input stream", "PointCloud.cpp");
        }
        inputStream.close();

        if(flag == POINT_BASED_FLAG)
        {
            // save points in my member variables
            m_pointsNumber = listLines.size();
            if(listLines.size() < 1)
            {
                throw ExceptionIcp("The input file is empty!", "PointCloud.cpp");
            }
            m_pointSet = Eigen::MatrixXd::Zero(3,m_pointsNumber);
            for (int i = 0; i < m_pointsNumber; i++)
            {
                int idxCoord = 0;
                std::stringstream stream(listLines[i]);
                float coord;
                while(stream >> coord)
                {
                    if(!stream) break;
                    if(idxCoord > 2)
                    {
                        throw ExceptionIcp("The input file should contain 1 point per line (x y z)", "PointCloud.cpp");
                    }
                    m_pointSet(idxCoord,i) = coord;
                    // std::cout << "Found coord " << idxCoord << ": " << coord << "\n";
                    idxCoord ++;
                }

            }
        }
        else if(flag == SURFACE_BASED_FLAG)
        {
            // save points in my member variables
            int numLines = listLines.size();
            if(listLines.size() < 1)
            {
                throw ExceptionIcp("The input file is empty!", "PointCloud.cpp");
            }
            m_pointsNumber = 3 * numLines;
            // std::cout << "numPts = " << m_pointsNumber << std::endl;
            m_pointSet = Eigen::MatrixXd::Zero(3,m_pointsNumber);
            int idxPS = 0;
            int idxLine = 0;
            while(idxPS < m_pointsNumber && idxLine < numLines)
            {
                int counterPts = 0;
                // std::cout << "idxLine = " << idxLine << "\n";
                int idxCoord = 0;
                std::stringstream stream(listLines[idxLine]);
                float coord;
                while(stream >> coord)
                {
                    if(!stream)
                    break;
                    m_pointSet(idxCoord,idxPS) = coord;
                    idxCoord++;
                    if (idxCoord > 2)
                    {
                        idxCoord = 0;
                        idxPS++;
                    }
                    counterPts++;
                }
                if(counterPts != 9)
                {
                    throw ExceptionIcp("The input file should contain 3 point per line (x1 y1 z1 x2 y2 z2 x3 y3 z3)", "PointCloud.cpp");
                }
                idxLine++;
            }

        }
        // std::cout << m_pointSet << "\n";

        std::cout << "Finished reading " << m_pointsNumber << " points\n";

    }
    // ************************************************************************
    PointCloud::~PointCloud()
    {
    }
    // ************************************************************************
    PointCloud::PointCloud(const PointCloud& in_pSrc)
    {
        m_pointSet = in_pSrc.m_pointSet;
        m_pointsNumber = in_pSrc.m_pointsNumber;

    }
    // ************************************************************************
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
        if(transfMat(3,3) != 1)
        {
            throw ExceptionIcp("The transformation matrix should be 4x4, [R,t] format", "PointCloud.cpp");
        }
        Eigen::MatrixXd ps_homog = m_pointSet.colwise().homogeneous();
        Eigen::MatrixXd ps_updated = (transfMat * ps_homog).block(0,0,3,m_pointsNumber);
        m_pointSet = ps_updated;
    }

}//namespace ICP_MPHYG02
