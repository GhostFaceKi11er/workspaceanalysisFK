#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <iostream>
#include <cmath>

//对2个end effector求逆解
class TWOTCPIK{
public:
    TWOTCPIK(dart::dynamics::SkeletonPtr skel, std::vector<Eigen::Isometry3d>& TCPTargetPositionsLeftAndRight, int maxiteration):  
    m_skel(skel), m_TCPTargetPositionsLeftAndRight(TCPTargetPositionsLeftAndRight), m_maxiteration(maxiteration){}

    //获得关节限位
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
                //std::cout << "joint " << i << " lowerLimit: " << lowerLimit << " upperLimit: " << upperLimit << std::endl;
                m_jointLimit.push_back(lowerLimit);
                m_jointLimit.push_back(upperLimit);
                m_jointsLimits.push_back(m_jointLimit);
            }
        }
        return m_jointsLimits;
    }

    //计算伪逆Jacobian
    Eigen::MatrixXd getPseodoInverseJacobian(Eigen::MatrixXd wholeJacobian, double alpha_steplenth){
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(wholeJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd singularValues = svd.singularValues();

        Eigen::MatrixXd singularValuesInv = Eigen::MatrixXd::Zero(wholeJacobian.cols(), wholeJacobian.rows());
        for (int i = 0; i < singularValues.size(); ++i) {
            if (singularValues[i] > 1e-3) { 
                singularValuesInv(i, i) = 1.0 / singularValues[i];
            }
            else {
                singularValuesInv(i, i) = 0.0; // 忽略非常小的奇异值，避免数值不稳定
                std::cout << "Warining! robot in singularity" << std::endl;
            }

            // 动态调整步长
            if (singularValues.minCoeff() < 1e-4) {
                alpha_steplenth *= 0.8; // 减小步长
            } else {
                alpha_steplenth *= 1.05; // 增大步长
            }
        }

        // 构建伪逆矩阵
        Eigen::MatrixXd J_pseudo_inverse = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
        return J_pseudo_inverse;
    }
    
    //计算四元数误差，输入TCP当前姿态的四元数和目标位姿的四元数，输出Eigen::Vector3d类的四元数误差
    Eigen::Vector3d computeRotationError(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
        Eigen::Quaterniond normalized_q1 = q1.normalized(); // 确保 q1 是单位四元数
        Eigen::Quaterniond normalized_q2 = q2.normalized(); // 确保 q2 是单位四元数

        Eigen::Quaterniond q_error = normalized_q1.inverse() * normalized_q2;

        // 确保四元数的正方向
        if (q_error.w() < 0) {
            q_error.coeffs() *= -1;
        }
        
        // 提取旋转角度
        double angle = 2 * std::acos(std::clamp(q_error.w(), -1.0, 1.0));

        // 提取旋转轴，注意归一化
        Eigen::Vector3d axis = q_error.vec();
        double axisNorm = axis.norm();
        if (axisNorm > 1e-6) { // 避免除以零
            axis /= axisNorm;
        } else {
            axis.setZero(); // 如果轴长度过小，设置为零向量
        }

        return angle * axis;
    }
    
    //对两只手同时求逆解
    bool getIK_diy(){
        std::cout << "Finding solution using DIY IK..." << std::endl;
        //chu shi hua zi tai
        m_skel->setPositions(Eigen::VectorXd(m_skel->getNumDofs()).setZero());
        Eigen::Isometry3d rightHandTf;
        Eigen::Isometry3d leftHandTf;
        double epsilon = 1e-8;
        double alpha_steplenth = 0.5;
        int iteration = 0;
        std::vector<std::vector<double>> jointsLimits = this->getJointsLimits();
        double count = 0;
        double previousErrorNorm;

        //末端执行器的目标位姿
        leftHandTf.translation() = m_TCPTargetPositionsLeftAndRight[0].translation();
        leftHandTf.linear() = m_TCPTargetPositionsLeftAndRight[0].linear();
        rightHandTf.translation() = m_TCPTargetPositionsLeftAndRight[1].translation();
        rightHandTf.linear() = m_TCPTargetPositionsLeftAndRight[1].linear();

        //末端执行器旋转矩阵的四元数
        Eigen::Matrix3d rotationMatrixLH = leftHandTf.linear(); // 获取旋转矩阵
        Eigen::Quaterniond quaternionLHtarget(rotationMatrixLH);      // 通过旋转矩阵构造四元数
        Eigen::Matrix3d rotationMatrixRH = rightHandTf.linear(); // 获取旋转矩阵
        Eigen::Quaterniond quaternionRHtarget(rotationMatrixRH);      // 通过旋转矩阵构造四元数

        while (iteration < m_maxiteration){
            //得到左手的雅克比矩阵
            Eigen::MatrixXd linearJacobianLH = m_skel->getLinearJacobian(m_skel->getBodyNode("leftHandEE")); //3*18
            Eigen::MatrixXd angularJacobianLH = m_skel->getAngularJacobian(m_skel->getBodyNode("leftHandEE")); //3*18
            Eigen::MatrixXd jacobianLH = Eigen::MatrixXd::Zero(6, 18);
            jacobianLH << linearJacobianLH, angularJacobianLH;
            //得到右手的雅克比矩阵
            Eigen::MatrixXd linearJacobianRH = m_skel->getLinearJacobian(m_skel->getBodyNode("rightHandEE")); //3*18
            Eigen::MatrixXd angularJacobianRH = m_skel->getAngularJacobian(m_skel->getBodyNode("rightHandEE")); //3*18
            Eigen::MatrixXd jacobianRH = Eigen::MatrixXd::Zero(6, 18);
            jacobianRH << linearJacobianRH, angularJacobianRH;

            //左TCP当前的旋转矩阵的四元数
            Eigen::Isometry3d LHeePose = m_skel->getBodyNode("leftHandEE")->getWorldTransform();
            Eigen::Matrix3d rotationMatrixLH = LHeePose.linear();
            Eigen::Quaterniond quaternionLHcurrent(rotationMatrixLH);
            //右TCP当前的旋转矩阵的四元数
            Eigen::Isometry3d RHeePose = m_skel->getBodyNode("rightHandEE")->getWorldTransform();
            Eigen::Matrix3d rotationMatrixRH = RHeePose.linear();
            Eigen::Quaterniond quaternionRHcurrent(rotationMatrixRH);            

            //计算左手的姿态误差
            Eigen::Vector3d LHrotation_error = this->computeRotationError(quaternionLHcurrent, quaternionLHtarget);
            Eigen::Vector6d poseErrorLH;
            Eigen::Vector3d RHrotation_error = this->computeRotationError(quaternionRHcurrent, quaternionRHtarget);
            Eigen::Vector6d poseErrorRH;
            // std::cout << "-----------------------" << std::endl;
            //得到关节配置
            Eigen::VectorXd theta = m_skel->getPositions();
            
            //计算位置误差
            Eigen::Vector3d RHtranslation_error = rightHandTf.translation() - RHeePose.translation();
            Eigen::Vector3d LHtranslation_error = leftHandTf.translation() - LHeePose.translation();
            
            // LHrotation_error = Eigen::Vector3d::Zero();
            // RHrotation_error = Eigen::Vector3d::Zero();
            poseErrorLH << LHtranslation_error, LHrotation_error;
            poseErrorRH << RHtranslation_error, RHrotation_error;
            Eigen::VectorXd totalError(12);
            totalError << poseErrorLH, poseErrorRH;

            // 计算误差，如果误差小于epsilon则判断关节角度是否在关节限位内，如果没有超限则停止迭代
            double errorNorm = totalError.norm();
            if (errorNorm <= epsilon){
                bool allWithinLimits = true;
                for (std::size_t i = 0; i < theta.size(); ++i) {
                    double jointAngle = theta[i];
                    double lowerLimit = jointsLimits[i][0];
                    double upperLimit = jointsLimits[i][1];
                    
                    if (jointAngle < lowerLimit || jointAngle > upperLimit) {
                        allWithinLimits = false;
                        iteration = 0;
                        std::cout << "Angle not within restriction. theta:" << theta << std::endl;
                        theta = Eigen::VectorXd(m_skel->getNumDofs()).setZero();
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
            if (iteration > 0 && errorNorm > 1.1 * previousErrorNorm) {
                alpha_steplenth *= 0.8;  // 减小步长变化
            } else if (iteration > 0 && errorNorm < 0.9 * previousErrorNorm) {
                alpha_steplenth *= 1.05; // 增大步长变化
            }
            alpha_steplenth = std::clamp(alpha_steplenth, 1e-4, 1.0);  // 限制步长范围

            //计算伪逆矩阵
            Eigen::MatrixXd J_pseudo_inverse_lh = getPseodoInverseJacobian(jacobianLH, alpha_steplenth);
            Eigen::MatrixXd J_pseudo_inverse_rh = getPseodoInverseJacobian(jacobianRH, alpha_steplenth);

            //计算关节增量, 更新关节配置
            Eigen::VectorXd delta_theta_lh = J_pseudo_inverse_lh * poseErrorLH;
            Eigen::VectorXd delta_theta_rh = J_pseudo_inverse_rh * poseErrorRH;
            Eigen::VectorXd delta_theta(18);
            delta_theta << (delta_theta_lh.segment(0,4)+delta_theta_rh.segment(0,4))*0.5, delta_theta_lh.segment(4,7), delta_theta_rh.segment(11,7);
            theta += alpha_steplenth * delta_theta;

            for (int i = 0; i < theta.size(); ++i) {
                theta[i] = std::fmod(theta[i], 2 * M_PI); // 先对 2*pi 取模
                if (theta[i] > M_PI) {
                    theta[i] -= 2 * M_PI; // 如果结果大于 pi，减去 2*pi
                } else if (theta[i] < -M_PI) {
                    theta[i] += 2 * M_PI; // 如果结果小于 -pi，加上 2*pi
                }
            }
            m_skel->setPositions(theta);
            //std::cout << "delta_theta_lh: " << delta_theta_lh.transpose() << std::endl << " delta_theta_rh: " << delta_theta_rh.transpose() << std::endl;
            if (iteration%10 == 0){
                std::cout << "iteration: " << iteration << " errorNorm.norm(): " << errorNorm << std::endl;
                // std::cout << "theta:" << theta << std::endl;
                //std::cout << "delta_theta_lh: " << delta_theta_lh.transpose() << std::endl << " delta_theta_rh: " << delta_theta_rh.transpose() << std::endl;
            }

            previousErrorNorm = errorNorm;
            iteration++;
            //std::cout << "totaliteration: " << totaliteration << " count: " << count << std::endl;
        }
    std::cout << "No solution found using DIY IK within the time limit." << std::endl;
    return false;
    }

    //对左手求逆解
    bool getIK_diy_leftarm(){
        std::cout << "Finding solution using DIY IK..." << std::endl;
        //chu shi hua zi tai
        std::vector<std::vector<double>> jointsLimits = this->getJointsLimits();
        m_skel->setPositions(Eigen::VectorXd(m_skel->getNumDofs()).setZero());
        Eigen::Isometry3d leftHandTf = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d rightHandTf = Eigen::Isometry3d::Identity();
        double epsilon = 1e-9;
        double alpha_steplenth = 0.3;
        //double alpha_steplenth = 0.001185;
        int iteration = 0;
        double count = 0;
        double previousErrorNorm;
        //设置末端执行器的目标位置
        leftHandTf.translation() = m_TCPTargetPositionsLeftAndRight[0].translation();
        leftHandTf.linear() = m_TCPTargetPositionsLeftAndRight[0].linear();
        Eigen::Matrix3d rotationMatrixTP = leftHandTf.linear(); // 获取旋转矩阵
        Eigen::Quaterniond quaternionLHtarget(rotationMatrixTP);      // 通过旋转矩阵构造四元数
        Eigen::VectorXd theta = m_skel->getPositions();
        std::cout << "leftHandTf.translation(): " << leftHandTf.translation() << " leftHandTf.linear(): " << leftHandTf.linear() << std::endl;
        

        while (iteration < m_maxiteration){
            Eigen::MatrixXd linearJacobian = m_skel->getLinearJacobian(m_skel->getBodyNode("leftHandEE")); //3*18
            Eigen::MatrixXd angularJacobian = m_skel->getAngularJacobian(m_skel->getBodyNode("leftHandEE")); //3*18
            Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 18);
            jacobian << linearJacobian, angularJacobian;
            //std::cout << "jacobian: " << jacobian << std::endl;
            Eigen::Isometry3d LHeePose = m_skel->getBodyNode("leftHandEE")->getWorldTransform();
            Eigen::Matrix3d rotationMatrixLH = LHeePose.linear();
            Eigen::Quaterniond quaternionLHcurrent(rotationMatrixLH);

            //计算姿态误差
            Eigen::Vector3d LHrotation_error = this->computeRotationError(quaternionLHcurrent, quaternionLHtarget);
            //计算位置误差
            Eigen::Vector3d LHtranslation_error = leftHandTf.translation() - LHeePose.translation();
            Eigen::Vector6d poseError;
            poseError << LHtranslation_error, LHrotation_error;

            //伪逆法
            Eigen::MatrixXd J_pseudo_inverse_lh = getPseodoInverseJacobian(jacobian, alpha_steplenth);


            // 计算误差，如果误差小于epsilon则判断关节角度是否在关节限位内，如果没有超限则停止迭代
            double errorNorm = poseError.norm();
            if(iteration%10 == 0){
                std::cout << "LHeePose.translation(): " << LHeePose.translation().transpose() << "  iteration: " << iteration << "  translation_error.norm(): " << LHtranslation_error.norm() << " LHrotation_error.norm: "<< LHrotation_error.norm() << std::endl;
            }
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
                    std::cout << "Solution found using DIY IK within the time limit and all joints within limits!" << std::endl;
                    std::cout << "theta: " << theta.transpose() << std::endl;
                    m_skel->setPositions(theta);
                    count++;
                    return true;
                }
                else {
                    iteration = 0;
                    theta = Eigen::VectorXd(m_skel->getNumDofs()).setZero();
                }
            }

            // //动态调整步长
            if (iteration > 0 && errorNorm > 1.1 * previousErrorNorm) {
                alpha_steplenth *= 0.8;  // 减小步长变化
            } else if (iteration > 0 && errorNorm < 0.9 * previousErrorNorm) {
                alpha_steplenth *= 1.05; // 增大步长变化
            }
            alpha_steplenth = std::clamp(alpha_steplenth, 0.000001, 1.0);  // 限制步长范围

            //计算关节增量, 更新关节配置
            Eigen::VectorXd delta_theta = J_pseudo_inverse_lh * poseError;
            //std::cout << "delta_theta_lh: " << delta_theta_lh.transpose() << std::endl;
            //std::cout << "delta_theta: " << delta_theta.transpose() << std::endl;
            theta += alpha_steplenth * delta_theta;


            for (int i = 0; i < theta.size(); ++i) {
                theta[i] = std::fmod(theta[i], 2 * M_PI); // 先对 2*pi 取模
                if (theta[i] > M_PI) {
                    theta[i] -= 2 * M_PI; // 如果结果大于 pi，减去 2*pi
                } else if (theta[i] < -M_PI) {
                    theta[i] += 2 * M_PI; // 如果结果小于 -pi，加上 2*pi
                }
            }
            m_skel->setPositions(theta);

            previousErrorNorm = errorNorm;
            iteration++;
        }
    std::cout << "No solution found using DIY IK within the time limit." << std::endl;
    return false;
    }

    //对右手求逆解
    bool getIK_diy_rightarm(){
        std::cout << "Finding solution using DIY IK..." << std::endl;
        //chu shi hua zi tai
        std::vector<std::vector<double>> jointsLimits = this->getJointsLimits();
        m_skel->setPositions(Eigen::VectorXd(m_skel->getNumDofs()).setZero());
        Eigen::Isometry3d leftHandTf = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d rightHandTf = Eigen::Isometry3d::Identity();
        double epsilon = 1e-9;
        double alpha_steplenth = 0.3;
        int iteration = 0;
        double count = 0;
        double previousErrorNorm;
        //设置末端执行器的目标位置
        rightHandTf.translation() = m_TCPTargetPositionsLeftAndRight[1].translation();
        rightHandTf.linear() = m_TCPTargetPositionsLeftAndRight[1].linear();
        Eigen::Matrix3d rotationMatrixTP = rightHandTf.linear(); // 获取旋转矩阵
        Eigen::Quaterniond quaternionRHtarget(rotationMatrixTP);      // 通过旋转矩阵构造四元数
        Eigen::VectorXd theta = m_skel->getPositions();
        std::cout << "rightHandTf.translation(): " << rightHandTf.translation() << " rightHandTf.linear(): " << rightHandTf.linear() << std::endl;
        

        while (iteration < m_maxiteration){
            // Eigen::MatrixXd wholejacobian = m_skel->getJacobian(m_skel->getBodyNode("rightHandEE"));
            Eigen::MatrixXd linearJacobian = m_skel->getLinearJacobian(m_skel->getBodyNode("rightHandEE")); //3*18
            Eigen::MatrixXd angularJacobian = m_skel->getAngularJacobian(m_skel->getBodyNode("rightHandEE")); //3*18
            // std::cout << "whole: " << wholejacobian << "\n---------------------------";
            Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 18);
            // jacobian << wholejacobian;
            jacobian << linearJacobian, angularJacobian;
            // std::cout << "whole2: " << jacobian << "\n";

            //std::cout << "jacobian: " << jacobian << std::endl;
            Eigen::Isometry3d RHeePose = m_skel->getBodyNode("rightHandEE")->getWorldTransform();
            Eigen::Matrix3d rotationMatrixRH = RHeePose.linear();
            Eigen::Quaterniond quaternionRHcurrent(rotationMatrixRH);

            //计算姿态误差
            Eigen::Vector3d RHrotation_error = this->computeRotationError(quaternionRHcurrent, quaternionRHtarget);
            //计算位置误差
            Eigen::Vector3d RHtranslation_error = rightHandTf.translation() - RHeePose.translation();
            Eigen::Vector6d poseError;
            poseError << RHtranslation_error, RHrotation_error;

            //伪逆法
            Eigen::MatrixXd J_pseudo_inverse_rh = getPseodoInverseJacobian(jacobian, alpha_steplenth);


            // 计算误差，如果误差小于epsilon则判断关节角度是否在关节限位内，如果没有超限则停止迭代
            double errorNorm = poseError.norm();
            if(iteration%10 == 0){
                std::cout << "RHeePose.translation(): " << RHeePose.translation().transpose() << "  iteration: " << iteration << "  translation_error.norm(): " << RHtranslation_error.norm() << " RHrotation_error.norm: "<< RHrotation_error.norm() << std::endl;
            }
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
                    std::cout << "Solution found using DIY IK within the time limit and all joints within limits!" << std::endl;
                    std::cout << "theta: " << theta.transpose() << std::endl;
                    m_skel->setPositions(theta);
                    count++;
                    return true;
                }
                else {
                    iteration = 0;
                    theta = Eigen::VectorXd(m_skel->getNumDofs()).setZero();
                }
            }

            // //动态调整步长
            if (iteration > 0 && errorNorm > 1.1 * previousErrorNorm) {
                alpha_steplenth *= 0.8;  // 减小步长变化
            } else if (iteration > 0 && errorNorm < 0.9 * previousErrorNorm) {
                alpha_steplenth *= 1.05; // 增大步长变化
            }
            alpha_steplenth = std::clamp(alpha_steplenth, 0.000001, 1.0);  // 限制步长范围

            //计算关节增量, 更新关节配置
            Eigen::VectorXd delta_theta = J_pseudo_inverse_rh * poseError;
            //std::cout << "delta_theta_lh: " << delta_theta_lh.transpose() << std::endl;
            //std::cout << "delta_theta: " << delta_theta.transpose() << std::endl;
            theta += alpha_steplenth * delta_theta;


            for (int i = 0; i < theta.size(); ++i) {
                theta[i] = std::fmod(theta[i], 2 * M_PI); // 先对 2*pi 取模
                if (theta[i] > M_PI) {
                    theta[i] -= 2 * M_PI; // 如果结果大于 pi，减去 2*pi
                } else if (theta[i] < -M_PI) {
                    theta[i] += 2 * M_PI; // 如果结果小于 -pi，加上 2*pi
                }
            }
            m_skel->setPositions(theta);

            previousErrorNorm = errorNorm;
            iteration++;
        }
    std::cout << "No solution found using DIY IK within the time limit." << std::endl;
    return false;
    }

private:
    dart::dynamics::SkeletonPtr m_skel;
    std::vector<Eigen::Isometry3d> m_TCPTargetPositionsLeftAndRight;
    int m_maxiteration;
};

//用dart中的方法进行2TCP求逆解
class InverseKinematics{
public:
    InverseKinematics(dart::dynamics::SkeletonPtr skel, std::vector<Eigen::Isometry3d>& eePositions, Eigen::VectorXd& jointPos, 
                        bool ifUseWholeBody, bool ifApply): m_skel(skel), m_eePositions(eePositions), m_jointPos(jointPos), 
                        m_ifUseWholeBody(ifUseWholeBody), m_ifApply(ifApply){}

    bool getIk(){  
        std::cout << "Finding solution using DART..." << std::endl;
        m_skel->setPositions(Eigen::VectorXd(m_skel->getNumDofs()).setZero());

        std::shared_ptr<dart::dynamics::InverseKinematics> leftHandIkPtr;
        std::shared_ptr<dart::dynamics::InverseKinematics> rightHandIkPtr;
        
        //设置右手的姿态，是否用全身关节去解以及是否应用关节配置
        rightHandIkPtr = m_skel->getBodyNode("rightHandEE")->getOrCreateIK(); //set a Ik pointer
        Eigen::Isometry3d rhtransform = Eigen::Isometry3d::Identity(); // Identity matrix for no rotation
        rhtransform.translation() = m_eePositions[1].translation();  // Set the translation (position)
        rhtransform.linear() = m_eePositions[1].linear();
        rightHandIkPtr->getTarget()->setTransform(rhtransform);
        if (m_ifUseWholeBody) {
            rightHandIkPtr->useWholeBody();
            } else {
            rightHandIkPtr->useChain();
        }

        //设置左手的姿态，是否用全身关节去解以及是否应用关节配置
        leftHandIkPtr = m_skel->getBodyNode("leftHandEE")->getOrCreateIK();
        Eigen::Isometry3d lhtransform = Eigen::Isometry3d::Identity(); // Identity matrix for no rotation
        lhtransform.linear() = rhtransform.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        lhtransform.translation() = m_eePositions[0].translation();  // Set the translation (position)
        leftHandIkPtr->getTarget()->setTransform(lhtransform);

        if (m_ifUseWholeBody) {
            leftHandIkPtr->useWholeBody();
        } else {
            leftHandIkPtr->useChain();
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
    std::vector<Eigen::Isometry3d> m_eePositions;
    Eigen::VectorXd m_jointPos;
    bool m_ifUseWholeBody;
    bool m_ifApply;
};


int main(){
    std::string urdfFile = "/home/haitaoxu/workspaceanalysis/models/M1/M1_full_load.urdf";
    dart::simulation::WorldPtr world = dart::simulation::World::create();
    dart::utils::DartLoader loader;
    auto skel = loader.parseSkeleton(urdfFile);

    //初始状态为零位
    skel->setPositions(Eigen::VectorXd(skel->getNumDofs()).setZero());

    //获取2个end effector的初始位姿
    Eigen::Isometry3d LHeePose = skel->getBodyNode("leftHandEE")->getWorldTransform();
    Eigen::Isometry3d RHeePose = skel->getBodyNode("rightHandEE")->getWorldTransform();

    //设置2个end effector的目标位姿
    LHeePose.translation()[0] += 0.6;
    RHeePose.translation()[0] += 0.6;
    LHeePose.translation()[2] -= 0.2;
    RHeePose.translation()[2] -= 0.2;
    std::vector<Eigen::Isometry3d> TCPTargetPositionsLeftAndRight = {LHeePose, RHeePose};
    std::cout << "LHeePose.translation(): " << LHeePose.translation() << " LHeePose.linear(): " << LHeePose.linear() << std::endl;
    std::cout << "RHeePose.translation(): " << RHeePose.translation() << " RHeePose.linear(): " << RHeePose.linear() << std::endl;

    //dart中的方法对2TCP求解
    bool ifUseWholeBody = false;
    bool ifApply = false;
    skel->setPositions(Eigen::VectorXd(skel->getNumDofs()).setZero());
    Eigen::VectorXd jointPos(skel->getNumDofs());
    InverseKinematics InverseKinematics(skel, TCPTargetPositionsLeftAndRight, jointPos, ifUseWholeBody, ifApply);
    bool IKsolution = InverseKinematics.getIk();
    
    //自定义方法求解
    int maxiteration = 1000;
    TWOTCPIK TWOTCPIK(skel, TCPTargetPositionsLeftAndRight, maxiteration);
    bool IKdiysolutionLeftandRight = TWOTCPIK.getIK_diy();
    
    std::cout << " dart get(IK): " << IKsolution << std::endl;
    std::cout << "TCPTargetPositionsLeftAndRight have IK solution for left hand and right hand: " << IKdiysolutionLeftandRight << std::endl;
    
    
    return 0;
}
