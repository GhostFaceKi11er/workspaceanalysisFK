#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include "SpaceSampler.h"
#include <iostream>
#include <random>

class TWOTCPIK{
public:
    TWOTCPIK(dart::dynamics::SkeletonPtr skel, dart::simulation::WorldPtr& world, std::vector<Eigen::Vector3d>& TCPTargetPositionsLeftAndRight, 
    int maxiteration):  m_skel(skel), m_world(world), m_TCPTargetPositionsLeftAndRight(TCPTargetPositionsLeftAndRight), m_maxiteration(maxiteration){}
    
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
                std::cout << "joint " << i << " lowerLimit: " << lowerLimit << " upperLimit: " << upperLimit << std::endl;
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
        jacobian = jacobian.block(0,0,3,4) + jacobian.block(0,11,3,7);
        return jacobian;
    }

    Eigen::MatrixXd getLHjacobian(){
        dart::dynamics::BodyNodePtr endEffector = m_skel->getBodyNode("leftHandEE");
        Eigen::MatrixXd jacobian = m_skel->getJacobian(endEffector); //获得的是6*18的矩阵，包含了整个机器人的活动关节
        jacobian = jacobian.topRows(3);
        jacobian = jacobian.block(0,0,3,11);

        return jacobian;
    }
    
    Eigen::VectorXd getRHtheta(){
        Eigen::VectorXd theta = m_skel->getPositions(); //大小为18
        theta = theta.segment(0,4) + theta.segment(11,7);
        return theta;
    }

    Eigen::VectorXd getLHtheta(){
        Eigen::VectorXd theta = m_skel->getPositions(); //大小为18
        theta = theta.segment(0,11);
        return theta;
    }

    Eigen::MatrixXd getPseodoInverseJacobian(Eigen::MatrixXd wholeJacobian){
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(wholeJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd singularValues = svd.singularValues();

        Eigen::MatrixXd singularValuesInv = Eigen::MatrixXd::Zero(wholeJacobian.cols(), wholeJacobian.rows());
        for (int i = 0; i < singularValues.size(); ++i) {
            if (singularValues[i] > 1e-3) { // 忽略非常小的奇异值，避免数值不稳定
                singularValuesInv(i, i) = 1.0 / singularValues[i];
            }
        }

        // 构建伪逆矩阵
        Eigen::MatrixXd J_pseudo_inverse = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
        return J_pseudo_inverse;
    }

    bool getIK_diy(){
        std::cout << "Finding solution using DIY IK..." << std::endl;
        //chu shi hua zi tai
        m_skel->setPositions(Eigen::VectorXd(m_skel->getNumDofs()).setZero());
        Eigen::Isometry3d rightHandTf;
        Eigen::Isometry3d leftHandTf;
        double epsilon = 1e-4;
        double alpha_steplenth = 0.01;
        int iteration = 0;
        std::vector<std::vector<double>> jointsLimits = this->getJointsLimits();
        double count = 0;
        double previousErrorNorm;
        //设置末端执行器的目标位置
        leftHandTf.translation() = m_TCPTargetPositionsLeftAndRight[0];
        rightHandTf.translation() = m_TCPTargetPositionsLeftAndRight[1];
        while (iteration < m_maxiteration){
            //得到左右手的雅克比矩阵
            Eigen::MatrixXd rhJacobian = getRHjacobian(); //3*11
            Eigen::MatrixXd lhJacobian = getLHjacobian(); //3*11

            //得到左右手的关节配置
            Eigen::VectorXd RHtheta = getRHtheta(); //3*11
            Eigen::VectorXd LHtheta = getLHtheta(); //3*11
            Eigen::VectorXd theta = m_skel->getPositions();

            Eigen::Isometry3d LHeePose = m_skel->getBodyNode("leftHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
            Eigen::Isometry3d RHeePose = m_skel->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
            
            //计算位置误差
            Eigen::Vector3d RHtranslation_error = rightHandTf.translation() - RHeePose.translation();
            Eigen::Vector3d LHtranslation_error = leftHandTf.translation() - LHeePose.translation();
            Eigen::VectorXd translation_error(6);
            translation_error << LHtranslation_error, RHtranslation_error;

            //计算伪逆矩阵
            //std::cout << "wholeJacobian.rows(): " << wholeJacobian.rows() << "  and .cols():"<< wholeJacobian.cols() << "translation_error.size(): " << translation_error.size() << std::endl;
            Eigen::MatrixXd J_pseudo_inverse_lh = getPseodoInverseJacobian(lhJacobian);
            Eigen::MatrixXd J_pseudo_inverse_rh = getPseodoInverseJacobian(rhJacobian);

            // 计算误差，如果误差小于epsilon则判断关节角度是否在关节限位内，如果没有超限则停止迭代
            double errorNorm = translation_error.norm();
            std::cout << "iteration: " << iteration << " translation_error.norm(): " << translation_error.norm() << std::endl;
            if (errorNorm <= epsilon){
                bool allWithinLimits = true;
                for (std::size_t i = 0; i < theta.size(); ++i) {
                    double jointAngle = theta[i];
                    double lowerLimit = jointsLimits[i][0];
                    double upperLimit = jointsLimits[i][1];
                    
                    if (jointAngle < lowerLimit || jointAngle > upperLimit) {
                        allWithinLimits = false;
                        iteration = 0;
                        break;
                    }
                }
            
                if (allWithinLimits) {          
                    std::cout << "Solution found using DIY IK within the time limit!" << std::endl;
                    m_skel->setPositions(theta);
                    count++;
                    return true;
                }
            }

            //动态调整步长
            // if (iteration > 0 && errorNorm > 1.1 * previousErrorNorm) {
            //     alpha_steplenth *= 0.8;  // 减小步长变化
            // } else if (iteration > 0 && errorNorm < 0.9 * previousErrorNorm) {
            //     alpha_steplenth *= 1.05; // 增大步长变化
            // }
            // alpha_steplenth = std::clamp(alpha_steplenth, 1e-4, 1.0);  // 限制步长范围

            //计算关节增量, 更新关节配置
            Eigen::VectorXd delta_theta_lh = J_pseudo_inverse_lh * LHtranslation_error;
            Eigen::VectorXd delta_theta_rh = J_pseudo_inverse_rh * RHtranslation_error;
            Eigen::VectorXd delta_theta(18);
            delta_theta << (delta_theta_lh.segment(0,4)+delta_theta_rh.segment(0,4))*0.5, delta_theta_lh.segment(4,7), delta_theta_rh.segment(11,7);
            theta += alpha_steplenth * delta_theta;
            m_skel->setPositions(theta);

            previousErrorNorm = errorNorm;
            iteration++;
            //std::cout << "totaliteration: " << totaliteration << " count: " << count << std::endl;
        }
    std::cout << "No solution found using DIY IK within the time limit." << std::endl;
    return false;
    }

private:
    dart::dynamics::SkeletonPtr m_skel;
    dart::simulation::WorldPtr m_world;
    std::vector<Eigen::Vector3d> m_TCPTargetPositionsLeftAndRight;
    int m_maxiteration;
};

class InverseKinematics{
public:
    InverseKinematics(dart::dynamics::SkeletonPtr skel, std::vector<Eigen::Vector3d>& eePositions, Eigen::VectorXd& jointPos, 
                        bool ifUseWholeBody, bool ifApply): m_skel(skel), m_eePositions(eePositions), m_jointPos(jointPos), 
                        m_ifUseWholeBody(ifUseWholeBody), m_ifApply(ifApply){}

    bool getIk(){  
        std::cout << "Finding solution using DART..." << std::endl;
        m_skel->setPositions(Eigen::VectorXd(m_skel->getNumDofs()).setZero());

        std::shared_ptr<dart::dynamics::InverseKinematics> leftHandIkPtr;
        std::shared_ptr<dart::dynamics::InverseKinematics> rightHandIkPtr;
        leftHandIkPtr = m_skel->getBodyNode("leftHandEE")->getOrCreateIK();
        Eigen::Isometry3d lhtransform = Eigen::Isometry3d::Identity(); // Identity matrix for no rotation
        lhtransform.translation() = m_eePositions[0];  // Set the translation (position)
        std::cout << "lhtransform.translation(): " << lhtransform.translation() << std::endl;
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

        if (!m_skel->getIK(true)->findSolution(m_jointPos)) {
            std::cout << "NO Solution found using DART" << std::endl;
            return false;
        }
        if (m_ifApply) m_skel->setPositions(m_jointPos);
        std::cout << "Solution found using DART!" << std::endl;
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

    skel->setPositions(Eigen::VectorXd(skel->getNumDofs()).setZero());
    Eigen::Isometry3d LHeePose = skel->getBodyNode("leftHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    Eigen::Isometry3d RHeePose = skel->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    Eigen::Vector3d LHposition= LHeePose.translation();
    Eigen::Vector3d RHposition= RHeePose.translation();
    LHposition[0] += 0.01;
    RHposition[0] += 0.01;

    // double x_min = 0.6, x_max = 0.8;
    // double y_min = -0.3, y_max = 0.3;
    // double z_min = 0.5, z_max = 0.7;
    
    // std::vector<double> samplebound = {x_min, x_max, y_min, y_max, z_min, z_max};


    //std::vector<Eigen::Vector3d> samplePoints = getRandomPoints(samplebound);
    std::vector<Eigen::Vector3d> samplePoints = {LHposition, RHposition};
    //std::vector<Eigen::Vector3d> samplePoints = {Eigen::Vector3d(0.5, 0.3, 0.5), Eigen::Vector3d(0.5, -0.3, 0.5)};
    for (auto position: samplePoints) std::cout << "position: " << position << std::endl;

    std::vector<Eigen::Vector3d> TCPTargetPositionsLeftAndRight = samplePoints;

    bool ifUseWholeBody = true;
    bool ifApply = true;
    skel->setPositions(Eigen::VectorXd(skel->getNumDofs()));
    Eigen::VectorXd jointPos(skel->getNumDofs());
    InverseKinematics InverseKinematics(skel, TCPTargetPositionsLeftAndRight, jointPos, ifUseWholeBody, ifApply);
    bool IKsolution = InverseKinematics.getIk();
    
    int maxiteration = 500;
    
    TWOTCPIK TWOTCPIK(skel, world, TCPTargetPositionsLeftAndRight, maxiteration);
    bool IKdiysolution = TWOTCPIK.getIK_diy();
    
    std::cout << " dart get(IK): " << IKsolution << std::endl;
    std::cout << "TCPTargetPositionsLeftAndRight have IK solution: " << IKdiysolution << std::endl;
    
    samplePoints.clear();

    // std::cout << " dart get(IK): " << IKsolution << std::endl;
    // std::cout << "TCPTargetPositionsLeftAndRight have IK solution: " << IKdiysolution << std::endl;
    
    return 0;
}
