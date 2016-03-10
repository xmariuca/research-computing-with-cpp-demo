#ifndef _H_ICP_CLASS
#define _H_ICP_CLASS

#include <Eigen/Dense>
#include <memory>
#include "PointCloud.h"


namespace ICP_MPHYG02
{
    class ICPRegistration
    {
    public:
        ICPRegistration(PointCloud& pFixed, PointCloud& qMoving, float errorThresh = 0.2, int maxIter = 50, float filterBadCorresp = 0.5 );
        void getLSEstimate(Eigen::Matrix4d& transfMatrix, double& out_err);


    private:
        PointCloud m_pFixed;
        PointCloud m_qMoving;
        float m_errorThreshold;
        int m_maxIterations;
        float m_filterBadCorresp;
        long m_numPoints;


    };
}//namespace ICP_MPHYG02
#endif
