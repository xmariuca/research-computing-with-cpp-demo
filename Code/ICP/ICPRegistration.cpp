#include "ICPRegistration.h"
#include <iostream>

namespace ICP_MPHYG02
{
    ICPRegistration::ICPRegistration(PointCloud& pFixed,
        PointCloud& qMoving,
        float errorThresh,
        int maxIter,
        float filterBadCorresp):m_pFixed(pFixed),m_qMoving(qMoving)
        {
            m_errorThreshold = errorThresh;
            m_maxIterations = maxIter;
            m_filterBadCorresp = filterBadCorresp;
            m_numPoints = m_pFixed.getPointsNum();
        }

        void ICPRegistration::getLSEstimate(Eigen::Matrix4d& transfMatrix, double& out_err)
        {
            Eigen::Vector3d pMeanVect(0,0,0);
            Eigen::Vector3d qMeanVect(0,0,0);

            Eigen::MatrixXd pCentered(Eigen::MatrixXd::Zero(3,m_numPoints));
            Eigen::MatrixXd qCentered(Eigen::MatrixXd::Zero(3,m_qMoving.getPointsNum()));

            m_pFixed.getMeanPCD(pMeanVect);
            m_qMoving.getMeanPCD(qMeanVect);

            m_pFixed.getCenteredPCD(pCentered);
            m_qMoving.getCenteredPCD(qCentered);
            // // compute the covariance matrix
            Eigen::Matrix3d covarianceMat(Eigen::Matrix3d::Zero());
            //// one version to compute the covariance matrix
            // for( int i = 0; i < m_numPoints; i++)
            // {
            //     covarianceMat += pCentered.col(i) * qCentered.col(i).transpose();
            // }
            //// faster covariance matrix computation!
            covarianceMat = (pCentered * qCentered.transpose()) *
            ((qCentered * qCentered.transpose()).inverse());

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(covarianceMat, Eigen::ComputeFullU | Eigen::ComputeFullV);

            Eigen::Matrix3d rotationMatrix = svd.matrixU() * svd.matrixV().transpose();

            Eigen::Vector3d translationVec = pMeanVect - rotationMatrix * qMeanVect;

            transfMatrix.block(0,0,3,3) = rotationMatrix;
            transfMatrix(3,3) = 1.0;
            transfMatrix.block(0,3,3,1) = translationVec;

            Eigen::MatrixXd errMat(rotationMatrix * qCentered + translationVec.replicate(1,m_numPoints) - pCentered);
            out_err = 0;
            for ( int i = 0; i < m_numPoints; i++)
            {
                out_err += errMat.col(i).squaredNorm();
            }
        }
    }    //namespace ICP_MPHYG02
