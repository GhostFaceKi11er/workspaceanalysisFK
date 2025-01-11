#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include "SpaceSampler.h"
#include <iostream>
#include <random>

class TWOTCPIK{
public:
    TWOTCPIK(dart::dynamics::SkeletonPtr skel, dart::simulation::WorldPtr& world, std::vector<Eigen::Vector3d>& TCPTargetPositionsLeftAndRight):  m_skel(skel), m_world(world), m_TCPTargetPositionsLeftAndRight(TCPTargetPositionsLeftAndRight){}
    
    std::vector<std::vector<double>> getJointsLimits() {
        std::vector<std::vector<double>> m_jointsLimits;
        std::vector<double> m_jointLimit;
        for (std::size_t i = 0; i < m_skel->getNumJoints(); ++i) {
            auto joint = m_skel->getJoint(i);

            // Check if the joint is a RevoluteJoint
            if (auto revoluteJoint = dynamic_cast<dart::dynamics::RevoluteJoint*>(joint)) {
                std::string jointName = revoluteJoint->getName();
                auto dof = revoluteJoint->getDof(0); // Revolute joints have only one DOF

                double lowerLimit = dof->getPositionLowerLimit();
                double upperLimit = dof->getPositionUpperLimit();
                m_jointLimit.push_back(lowerLimit);
                m_jointLimit.push_back(upperLimit);
                m_jointsLimits.push_back(m_jointLimit);
            }
        }
        return m_jointsLimits;
    }

    Eigen::MatrixXd getRHjacobian(){
        dart::dynamics::BodyNodePtr endEffector = m_skel->getBodyNode("rightHandEE");
        Eigen::MatrixXd jacobian = m_skel->getJacobian(endEffector); //获得的是6*18的矩阵，包含了整个机器人的活动关节
        jacobian = jacobian.topRows(3);
        return jacobian;
    }

    Eigen::MatrixXd getLHjacobian(){
        dart::dynamics::BodyNodePtr endEffector = m_skel->getBodyNode("leftHandEE");
        Eigen::MatrixXd jacobian = m_skel->getJacobian(endEffector); //获得的是6*18的矩阵，包含了整个机器人的活动关节
        jacobian = jacobian.topRows(3);

        return jacobian;
    }
    
    Eigen::VectorXd getRHtheta(){
        Eigen::VectorXd theta = m_skel->getPositions(); //大小为18
        return theta;
    }

    Eigen::VectorXd getLHtheta(){
        Eigen::VectorXd theta = m_skel->getPositions(); //大小为18
        return theta;
    }

    bool getIK_diy(){
        //std::cout << "Finding solution using IK..." << std::endl;
        Eigen::Isometry3d rightHandTf;
        Eigen::Isometry3d leftHandTf;
        double epsilon = 1e-4;
        double alpha_steplenth = 0.7;
        int iteration = 0;
        std::vector<std::vector<double>> jointsLimits = this->getJointsLimits();

        //设置末端执行器的目标位置
        leftHandTf.translation() = m_TCPTargetPositionsLeftAndRight[0];
        rightHandTf.translation() = m_TCPTargetPositionsLeftAndRight[1];
        
        Eigen::MatrixXd rhJacobian = getRHjacobian(); //3*18
        Eigen::MatrixXd lhJacobian = getLHjacobian(); //3*18

        Eigen::VectorXd RHtheta = getRHtheta();
        Eigen::VectorXd LHtheta = getLHtheta();
        // Eigen::VectorXd wholetheta(LHtheta.size()+RHtheta.size());
        // wholetheta << LHtheta, RHtheta;
        Eigen::VectorXd wholetheta(LHtheta.size());
        //std::cout << "wholetheta.size(): " << wholetheta.size() << std::endl;

        while (iteration < 2000){
            Eigen::Isometry3d LHeePose = m_skel->getBodyNode("leftHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
            Eigen::Isometry3d RHeePose = m_skel->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
            //计算位置误差
            Eigen::Vector3d RHtranslation_error = rightHandTf.translation() - RHeePose.translation();
            Eigen::Vector3d LHtranslation_error = leftHandTf.translation() - LHeePose.translation();
            //std::cout << "RHtranslation_error: " << RHtranslation_error << std::endl;
            Eigen::VectorXd translation_error(6);
            translation_error << LHtranslation_error, RHtranslation_error;

            Eigen::MatrixXd wholeJacobian(lhJacobian.rows()+rhJacobian.rows(), lhJacobian.cols()); //6*18
            wholeJacobian << lhJacobian, rhJacobian;
            //std::cout << "wholeJacobian.rows(): " << wholeJacobian.rows() << "  and .cols():"<< wholeJacobian.cols() << "translation_error.size(): " << translation_error.size() << std::endl;
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(wholeJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::MatrixXd J_pseudo_inverse = svd.solve(Eigen::MatrixXd::Identity(wholeJacobian.rows(), wholeJacobian.rows()));

            //std::cout << "J_pseudo_inverse.rows() and .cols(): " << J_pseudo_inverse.rows() << "  :"<< J_pseudo_inverse.cols() << "translation_error.size(): " << translation_error.size() << std::endl;
            // 计算关节增量
            Eigen::VectorXd delta_theta = J_pseudo_inverse * translation_error;
            wholetheta += alpha_steplenth * delta_theta;
            // Eigen::VectorXd theta(skel->getPositions().size());
            // theta = wholetheta.segment(0,11) + wholetheta.segment(14, 7);
            //std::cout << "wholetheta.size(): " << wholetheta.size() << std::endl;
            m_skel->setPositions(wholetheta);
            //std::cout << "delta_theta.norm(): " << delta_theta.norm() << std::endl;
            if (delta_theta.norm() <= epsilon){
                bool allWithinLimits = true;
                for (std::size_t i = 0; i < wholetheta.size(); ++i) {
                    double jointAngle = wholetheta[i];
                    double lowerLimit = jointsLimits[i][0];
                    double upperLimit = jointsLimits[i][1];
                    
                    if (jointAngle < lowerLimit || jointAngle > upperLimit) {
                        allWithinLimits = false;
                        break;
                    }
                }
                
                if (allWithinLimits) {
                    std::cout << "Solution found within the time limit!" << std::endl;
                    return true;
                }
            }
            iteration++;
        }
    //std::cout << "No solution found within the time limit." << std::endl;
    return false;
    }

private:
    dart::dynamics::SkeletonPtr m_skel;
    dart::simulation::WorldPtr m_world;
    std::vector<Eigen::Vector3d> m_TCPTargetPositionsLeftAndRight;
};

class InverseKinematics{
public:
    InverseKinematics(dart::dynamics::SkeletonPtr skel, std::vector<Eigen::Vector3d>& eePositions, Eigen::VectorXd& jointPos, 
                        bool ifUseWholeBody, bool ifApply): m_skel(skel), m_eePositions(eePositions), m_jointPos(jointPos), 
                        m_ifUseWholeBody(ifUseWholeBody), m_ifApply(ifApply){}

    bool getIk(){  
        //std::cout << "Finding solution using DART..." << std::endl;
        std::shared_ptr<dart::dynamics::InverseKinematics> headIkPtr;
        std::shared_ptr<dart::dynamics::InverseKinematics> leftHandIkPtr;
        std::shared_ptr<dart::dynamics::InverseKinematics> rightHandIkPtr;
        leftHandIkPtr = m_skel->getBodyNode("leftHandEE")->getOrCreateIK();
        Eigen::Isometry3d lhtransform = Eigen::Isometry3d::Identity(); // Identity matrix for no rotation
        lhtransform.translation() = m_eePositions[0];  // Set the translation (position)
        leftHandIkPtr->getTarget()->setTransform(lhtransform);
        if (m_ifUseWholeBody) {
            leftHandIkPtr->useWholeBody();
        } else {
            leftHandIkPtr->useChain();
        }
        rightHandIkPtr = m_skel->getBodyNode("rightHandEE")->getOrCreateIK(); //set a Ik pointer
        Eigen::Isometry3d rhtransform = Eigen::Isometry3d::Identity(); // Identity matrix for no rotation
        rhtransform.translation() = m_eePositions[1];  // Set the translation (position)
        rightHandIkPtr->getTarget()->setTransform(rhtransform);
        if (m_ifUseWholeBody) {
            rightHandIkPtr->useWholeBody();
            } else {
            rightHandIkPtr->useChain();
        }
        if (!m_skel->getIK(true)->findSolution(m_jointPos)) return false;
        if (m_ifApply) m_skel->setPositions(m_jointPos);
        //std::cout << "Solution found using DART!" << std::endl;
        return true;
    }

private:
    dart::dynamics::SkeletonPtr m_skel;
    std::vector<Eigen::Vector3d> m_eePositions;
    Eigen::VectorXd m_jointPos;
    bool m_ifUseWholeBody;
    bool m_ifApply;
};

std::vector<Eigen::Vector3d> getRandomPoints(std::vector<double> samplebound){
    std::vector<Eigen::Vector3d> samplePoints;
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int i = 0; i < 2; ++i) {
        double x = std::uniform_real_distribution<>(samplebound[0], samplebound[1])(gen);
        double y = std::uniform_real_distribution<>(samplebound[2], samplebound[3])(gen);
        double z = std::uniform_real_distribution<>(samplebound[4], samplebound[5])(gen);
        samplePoints.emplace_back(x, y, z);
    }
    return samplePoints;
}

int main(){
    std::string urdfFile = "/home/haitaoxu/workspaceanalysis/models/M1/M1_full_load.urdf";
    dart::simulation::WorldPtr world = dart::simulation::World::create();
    dart::utils::DartLoader loader;
    auto skel = loader.parseSkeleton(urdfFile);
    skel->setPositions(Eigen::VectorXd(skel->getNumDofs()));

    double x_min = 0.6, x_max = 0.8;
    double y_min = -0.3, y_max = 0.3;
    double z_min = 0.5, z_max = 0.7;
    
    std::vector<double> samplebound = {x_min, x_max, y_min, y_max, z_min, z_max};
    while(true){
        std::vector<Eigen::Vector3d> samplePoints = getRandomPoints(samplebound);
    
        std::vector<Eigen::Vector3d> TCPTargetPositionsLeftAndRight = samplePoints;
    
        TWOTCPIK TWOTCPIK(skel, world, TCPTargetPositionsLeftAndRight);

        bool ifUseWholeBody = true;
        bool ifApply = true;
        skel->setPositions(Eigen::VectorXd(skel->getNumDofs()));
        Eigen::VectorXd jointPos(skel->getNumDofs());
        InverseKinematics InverseKinematics(skel, TCPTargetPositionsLeftAndRight, jointPos, ifUseWholeBody, ifApply);
        bool IKsolution = InverseKinematics.getIk();
        bool IKdiysolution = TWOTCPIK.getIK_diy();
        if (IKsolution != IKdiysolution) {
            std::cout << " dart get(IK): " << IKsolution << std::endl;
            std::cout << "TCPTargetPositionsLeftAndRight have IK solution: " << IKdiysolution << std::endl;
            return 0;
        }
        samplePoints.clear();

    }
    // std::cout << " dart get(IK): " << IKsolution << std::endl;
    // std::cout << "TCPTargetPositionsLeftAndRight have IK solution: " << IKdiysolution << std::endl;
    
    return 0;
}
