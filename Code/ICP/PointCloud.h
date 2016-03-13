#ifndef _H_PCD_CLASS
#define _H_PCD_CLASS

#include <Eigen/Dense>

namespace ICP_MPHYG02
{
    class PointCloud
    {
    public:
        // PointCloud();
        PointCloud(Eigen::MatrixXd& in_pSet);
        // flag for the diff formats of the file
        PointCloud(std::string filename, bool flag);
        ~PointCloud();
        PointCloud(const PointCloud& in_pSrc);
        PointCloud& operator= (const PointCloud &in_pSrc);
        void getCenteredPCD(Eigen::MatrixXd& out_centeredPCD);
        void getMeanPCD(Eigen::Vector3d& out_mean);
        void applyTransformation(Eigen::Matrix4d& transfMat);
        long getPointsNum() { return m_pointsNumber;}
        void getPointSet(Eigen::MatrixXd& out_ps){out_ps = m_pointSet;}
        void setPointSet(Eigen::MatrixXd& in_ps){ m_pointSet = in_ps;}
        // void updatePositions(Eigen::MatrixXd& idx)
        // bool writePCD(std::string filename);

    private:
        Eigen::MatrixXd m_pointSet;
        long m_pointsNumber;

    };
}//namespace ICP_MPHYG02
#endif
