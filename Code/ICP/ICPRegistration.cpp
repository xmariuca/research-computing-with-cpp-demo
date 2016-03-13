#include "ICPRegistration.h"
#include <iostream>
#include <Eigen/Geometry>


namespace ICP_MPHYG02
{
    ICPRegistration::ICPRegistration(bool ICP_FLAG, int maxIter, float errorThresh, float filterBadCorresp)
    {
        m_errorThreshold = errorThresh;
        m_maxIterations = maxIter;
        m_filterBadCorresp = filterBadCorresp;
        m_ICP_FLAG = ICP_FLAG;
    }

    void ICPRegistration::getLSEstimate(PointCloud& pFixedPCD, PointCloud& qMovingPCD, Eigen::Matrix4d& out_transfMatrix, double& out_err)
    {
        long numPointsP = pFixedPCD.getPointsNum();
        long numPointsQ = qMovingPCD.getPointsNum();

        Eigen::Vector3d pMeanVect(0,0,0);
        Eigen::Vector3d qMeanVect(0,0,0);
        Eigen::MatrixXd pCentered(Eigen::MatrixXd::Zero(3,numPointsP));
        Eigen::MatrixXd qCentered(Eigen::MatrixXd::Zero(3,numPointsQ));
        Eigen::Matrix3d covarianceMat(Eigen::Matrix3d::Zero());

        // std::cout << "Demean pointclouds..." << std::endl;
        pFixedPCD.getMeanPCD(pMeanVect);
        qMovingPCD.getMeanPCD(qMeanVect);
        pFixedPCD.getCenteredPCD(pCentered);
        qMovingPCD.getCenteredPCD(qCentered);
        // // compute the covariance matrix
        //// one version to compute the covariance matrix
        // for( int i = 0; i < m_numPoints; i++)
        // {
        //     covarianceMat += pCentered.col(i) * qCentered.col(i).transpose();
        // }
        //// faster covariance matrix computation!
        // std::cout << "Compute covariance matrix..." << std::endl;

        // std::cout << "size pCentered " << pCentered.rows() << "\t" << pCentered.cols() << std::endl;
        // std::cout << "size qCentered " << qCentered.rows() << "\t" << qCentered.cols() << std::endl;

        covarianceMat = (pCentered * qCentered.transpose()) *
        ((qCentered * qCentered.transpose()).inverse());

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(covarianceMat, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // std::cout << "Compute rotation and translation..." << std::endl;

        Eigen::Matrix3d rotationMatrix = svd.matrixU() * svd.matrixV().transpose();

        Eigen::Vector3d translationVec = pMeanVect - rotationMatrix * qMeanVect;

        out_transfMatrix.block(0,0,3,3) = rotationMatrix;
        out_transfMatrix(3,3) = 1.0;
        out_transfMatrix.block(0,3,3,1) = translationVec;

        // std::cout << "Compute error mat..." << std::endl;
        Eigen::MatrixXd errMat(rotationMatrix * qCentered + translationVec.replicate(1,numPointsP) - pCentered);
        out_err = 0;
        for ( int i = 0; i < numPointsP; i++)
        {
            out_err += errMat.col(i).squaredNorm();
        }
    }

    void ICPRegistration::findCorrespondences(PointCloud& pFixedPCD, PointCloud& qMovingPCD,PointCloud& out_pFixedPCD, Eigen::MatrixXd& idxCorresp)
    {

        long numPointsP = pFixedPCD.getPointsNum();
        long numPointsQ = qMovingPCD.getPointsNum();

        idxCorresp = Eigen::MatrixXd::Zero(numPointsQ,2);
        Eigen::MatrixXd distanceMat = Eigen::MatrixXd::Zero(numPointsP,numPointsQ);

        Eigen::MatrixXd pFixed;
        Eigen::MatrixXd qMoving;
        pFixedPCD.getPointSet(pFixed);
        qMovingPCD.getPointSet(qMoving);

        Eigen::Vector3d thisP, thisQ;
        for(int i = 0; i < pFixed.cols(); i++)
        {
            thisP = pFixed.col(i);

            for(int j = 0; j < qMoving.cols(); j++)
            {
                thisQ = qMoving.col(j);
                distanceMat(i,j) = (thisP - thisQ).norm();
            }
        }
        // get min idx per column
        // for each q - find the closest neighbour in p
        // int idx = 0;
        Eigen::MatrixXd out_pFixed(Eigen::MatrixXd::Zero(3,numPointsP));
        int idx = 0;
        for(int i = 0; i < qMoving.cols(); i++)
        {
            Eigen::MatrixXd::Index minIdx;
            float minDist = distanceMat.col(i).minCoeff(&minIdx);
            idxCorresp(i,0) = i;
            idxCorresp(i,1) = minIdx;
            // std::cout << "minDist: \n" << minDist << "\n";
            // std::cout << "minIdx: \n" << minIdx << "\n";
            out_pFixed.col(idx) = pFixed.col(minIdx);
            idx ++;
        }
        // std::cout <<  "idxCorrespMat" << idxCorresp << std::endl;
        // return p in the correct order
        out_pFixedPCD.setPointSet(out_pFixed);
        // std::cout <<  " Corresp done!" << std::endl;
    }
    void ICPRegistration::solve(PointCloud& pFixedPCD, PointCloud& qMovingPCD, double& out_err, std::string& outputPath)
    {
        if(m_ICP_FLAG == POINT_BASED_FLAG)
        {
            Eigen::Matrix4d transfMatrix(Eigen::Matrix4d::Zero());
            printMessage("Estimating transformation...");
            ICPRegistration::getLSEstimate(pFixedPCD, qMovingPCD, transfMatrix, out_err);
            std::cout<< "*** Transf Matrix: \n" << transfMatrix << std::endl;
            std::cout<< "*** RMS error: " << out_err << std::endl;
            printMessage("Writing transformation to file...");
            writeToFile(transfMatrix, outputPath);
            printMessage("Done!");
        }
        else if(m_ICP_FLAG == SURFACE_BASED_FLAG)
        {
            Eigen::MatrixXd idxCorresp;
            Eigen::MatrixXd old_q;
            Eigen::MatrixXd updated_q;
            long numPointsQ = qMovingPCD.getPointsNum();
            Eigen::Matrix4d transfMatrix(Eigen::Matrix4d::Zero());
            Eigen::Matrix4d finalTransfMat(Eigen::Matrix4d::Identity());
            PointCloud out_pPCD(pFixedPCD);

            printMessage("Estimating transformation...");
            double thisErr = 0.0, prevErr = 0.0;
            int i = 0; //curent iteration
            for( i = 0; i <= m_maxIterations; i++ )
            {
                thisErr = 0;
                std::cout << "* ICP iter " << i+1 << std::endl;
                ICPRegistration::findCorrespondences(pFixedPCD, qMovingPCD, out_pPCD, idxCorresp);

                ICPRegistration::getLSEstimate(out_pPCD, qMovingPCD, transfMatrix, thisErr);

                // update q according to the transformation matrix
                qMovingPCD.getPointSet(old_q);
                Eigen::MatrixXd old_q_homog = old_q.colwise().homogeneous();
                updated_q = (transfMatrix * old_q_homog).block(0,0,3,numPointsQ);
                qMovingPCD.setPointSet(updated_q);

                finalTransfMat = transfMatrix * finalTransfMat;
                std::cout << "current RMS \t" << thisErr << std::endl;

                double diffErr = std::abs(thisErr - prevErr);
                if(diffErr < m_errorThreshold) break;

                prevErr = thisErr;
            }

            std::cout << "*** Final Transf = \n" << finalTransfMat << std::endl;
            std::cout << "*** ICP - num iterations = " << i+1 << std::endl;
            std::cout << "*** ICP - final error = " << thisErr << std::endl;
            printMessage("Writing transformation to file...");
            writeToFile(finalTransfMat, outputPath);
            printMessage("Done!");

        }
    }
}    //namespace ICP_MPHYG02
