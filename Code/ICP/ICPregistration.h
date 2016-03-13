#ifndef _H_ICP_CLASS
#define _H_ICP_CLASS

#include <Eigen/Dense>
#include <memory>
#include <string>
#include "PointCloud.h"
#include "defs.h"
#include "utils.h"

namespace ICP_MPHYG02
{
    class ICPRegistration
    {
    public:
        ICPRegistration(bool ICP_FLAG = SURFACE_BASED_FLAG, int maxIter = 30, float errorThresh = 0.000001, float filterBadCorresp = 0.5);
        void getLSEstimate(PointCloud& pFixedPCD, PointCloud& qMovingPCD, Eigen::Matrix4d& out_transfMatrix, double& out_err);
        void findCorrespondences(PointCloud& pFixedPCD, PointCloud& qMovingPCD,PointCloud& out_pFixedPCD, Eigen::MatrixXd& idxCorresp);
        void solve(PointCloud& pFixedPCD, PointCloud& qMovingPCD, double& out_err, Eigen::Matrix4d& out_transfMatrix,std::string& outputPath);

    private:
        bool m_ICP_FLAG;
        float m_errorThreshold;
        int m_maxIterations;
        float m_filterBadCorresp;


    };
}//namespace ICP_MPHYG02
#endif
