#include "robot/RobotSystem.h"

namespace WorkSpaceAnalysis {
RobotSystem::RobotSystem(const std::string& urdfFile) : m_urdfFile(urdfFile) { this->ConfigRobot(); }

RobotSystem::~RobotSystem() {}

void RobotSystem::ConfigRobot() {
    dart::utils::DartLoader urdfLoader;
    m_skel = urdfLoader.parseSkeleton(m_urdfFile);

    // set fix base
    m_skel->getRootBodyNode()->changeParentJointType<dart::dynamics::WeldJoint>();

    for (int i = 0; i < m_skel->getNumJoints(); ++i) {
        dart::dynamics::JointPtr joint = m_skel->getJoint(i);
        if (joint->getType() != "WeldJoint") {
            m_jointId[joint->getName()] = joint;
        }
    }

    for (int i = 0; i < m_skel->getNumBodyNodes(); ++i) {
        dart::dynamics::BodyNodePtr bn = m_skel->getBodyNode(i);
        m_linkId[bn->getName()] = bn;
    }

    m_dof = m_skel->getNumDofs();

    m_robotConfig.Resize(m_dof);

    m_robotConfig.jointPosLowerLimits = m_skel->getPositionLowerLimits().segment(0, m_dof); //Eigen 库中的一个函数，用于从向量中提取子向量。从index=0开始,骨架的关节下限位置向量中提取前 m_dof 个关节的下限位置。
    m_robotConfig.jointPosUpperLimits = m_skel->getPositionUpperLimits().segment(0, m_dof); //why
    m_robotConfig.jointVelLimits = m_skel->getVelocityUpperLimits();
    m_robotConfig.jointTorqueLimits = m_skel->getForceUpperLimits();
}

void RobotSystem::SetPositionLimits(const Eigen::VectorXd& posLowerLimits, const Eigen::VectorXd& posUpperLimits) {
    if (posLowerLimits.size() != m_dof) {
        printf("RobotSystem.SetPositionLimits: Failed to set position lower limits, size doesn't match, %d vs %d\n",
               (int)posLowerLimits.size(), m_dof);
        return;
    }
    if (posUpperLimits.size() != m_dof) {
        printf("RobotSystem.SetPositionLimits: Failed to set position upper limits, size doesn't match, %d vs %d\n",
               (int)posUpperLimits.size(), m_dof);
        return;
    }
    // m_skel->setPositionLowerLimits(posLowerLimits);
    // m_skel->setPositionUpperLimits(posUpperLimits);
    m_robotConfig.jointPosLowerLimits = posLowerLimits;
    m_robotConfig.jointPosUpperLimits = posUpperLimits;
}

void RobotSystem::SetVelocityLimits(const Eigen::VectorXd& velLimits) {
    if (velLimits.size() != m_dof) {
        printf("RobotSystem.SetVelocityLimits: Failed to set velocity limits, size doesn't match, %d vs %d\n",
               (int)velLimits.size(), m_dof);
        return;
    }
    // todo: whether to update in m_skel
    // m_skel->setVelocityLowerLimits(-velLimits);
    // m_skel->setVelocityLowerLimits(velLimits);
    m_robotConfig.jointVelLimits = velLimits;
}

void RobotSystem::SetAccelerationLimits(const Eigen::VectorXd& accLimits) {
    if (accLimits.size() != m_dof) {
        printf("RobotSystem.SetAccelerationLimits: Failed to set acceleration limits, size doesn't match, %d vs %d\n",
               (int)accLimits.size(), m_dof);
        return;
    }
    m_robotConfig.jointAccLimits = accLimits;
}

void RobotSystem::SetJerkLimits(const Eigen::VectorXd& jerkLimits) {
    if (jerkLimits.size() != m_dof) {
        printf("RobotSystem.SetJerkLimits: Failed to set jerk limits, size doesn't match, %d vs %d\n",
               (int)jerkLimits.size(), m_dof);
        return;
    }
    m_robotConfig.jointJerkLimits = jerkLimits;
}

void RobotSystem::SetTorqueLimits(const Eigen::VectorXd& torqueLimits) {
    if (torqueLimits.size() != m_dof) {
        printf("RobotSystem.SetTorqueLimits: Failed to set torque limits, size doesn't match, %d vs %d\n",
               (int)torqueLimits.size(), m_dof);
        return;
    }
    m_robotConfig.jointTorqueLimits = torqueLimits;
}

void RobotSystem::SetTcpOffset(EEndEffectorType type, const Eigen::Isometry3d& offset) { //需要抽象
    if (type == EEndEffectorType::LEFTHAND) m_skel->getJoint("leftArmEEJoint")->setTransformFromParentBodyNode(offset);
    if (type == EEndEffectorType::RIGHTHAND)
        m_skel->getJoint("rightArmEEJoint")->setTransformFromParentBodyNode(offset);
}

void RobotSystem::SetTcpPayload(EEndEffectorType type, const Eigen::VectorXd& parameters) { //需要抽象
    if (parameters.size() != 10) {
        printf("RobotSystem.SetTcpPayload: wrong size of input parameters");
        return;
    }
    if (type == EEndEffectorType::LEFTHAND) {
        m_linkId["leftHandEE"]->setMass(parameters(0));
        m_linkId["leftHandEE"]->setLocalCOM(parameters.segment(1, 3));
        m_linkId["leftHandEE"]->setMomentOfInertia(parameters(4), parameters(5), parameters(6), parameters(7),
                                                   parameters(8), parameters(9));
    }
    if (type == EEndEffectorType::RIGHTHAND) {
        m_linkId["rightHandEE"]->setMass(parameters(0));
        m_linkId["rightHandEE"]->setLocalCOM(parameters.segment(1, 3));
        m_linkId["rightHandEE"]->setMomentOfInertia(parameters(4), parameters(5), parameters(6), parameters(7),
                                                    parameters(8), parameters(9));
    }
}


void RobotSystem::UpdateSystem(const std::map<std::string, double>& jointPos,
                               const std::map<std::string, double>& jointVel, const Eigen::Vector3d& basePos,
                               const Eigen::Quaternion<double>& baseQuat, bool ifUpdateDynamics) {
    Eigen::Isometry3d basePose;
    basePose.linear() = baseQuat.normalized().toRotationMatrix();
    basePose.translation() = basePos;

    m_skel->getRootJoint()->setTransformFromParentBodyNode(basePose);

    for (std::map<std::string, double>::const_iterator it = jointPos.begin(); it != jointPos.end(); it++) {
        m_jointId.find(it->first)->second->setPosition(0, it->second);
        m_robotState.jointPositions[this->GetJointIdx(it->first)] = it->second;
    }

    for (std::map<std::string, double>::const_iterator it = jointVel.begin(); it != jointVel.end(); it++) {
        m_jointId.find(it->first)->second->setVelocity(0, it->second);
        m_robotState.jointVelocities[this->GetJointIdx(it->first)] = it->second;
    }

    // m_skel->computeForwardKinematics(); // this computes automatically
    if (ifUpdateDynamics) m_skel->computeForwardDynamics();
}

void RobotSystem::UpdateSystem(const Eigen::VectorXd& jointPos, const Eigen::VectorXd& jointVel,
                               const Eigen::Vector3d& basePos, const Eigen::Quaternion<double>& baseQuat,
                               bool ifUpdateDynamics) {
    m_robotState.jointPositions = jointPos;
    m_robotState.jointVelocities = jointVel;

    Eigen::Isometry3d basePose;
    basePose.linear() = baseQuat.normalized().toRotationMatrix();
    basePose.translation() = basePos;

    m_skel->getRootJoint()->setTransformFromParentBodyNode(basePose);

    m_skel->setPositions(jointPos);
    m_skel->setVelocities(jointVel);

    // m_skel->computeForwardKinematics(); // this computes automatically
    if (ifUpdateDynamics) m_skel->computeForwardDynamics();
}

int RobotSystem::InverseKinematics(EEndEffectorType type, Eigen::Isometry3d& pose, Eigen::VectorXd& jointPos,
                                   bool ifUseWholeBody, bool ifApply) {
    std::shared_ptr<dart::dynamics::InverseKinematics> ikPtr;
    if (type == EEndEffectorType::HEAD) ikPtr = m_linkId["headEE"]->getOrCreateIK();
    if (type == EEndEffectorType::LEFTHAND) ikPtr = m_linkId["leftHandEE"]->getOrCreateIK();
    if (type == EEndEffectorType::RIGHTHAND) ikPtr = m_linkId["rightHandEE"]->getOrCreateIK();

    ikPtr->getTarget()->setTransform(pose);

    if (ifUseWholeBody)
        ikPtr->useWholeBody();
    else
        ikPtr->useChain(); //逆运动学求解器会将目标 BodyNode 以及其所有祖先节点（从目标节点回溯到基座的节点）组合成一个运动学链，用于逆运动学计算。
    if (!ikPtr->findSolution(jointPos)) return -1;
    if (ifApply) ikPtr->setPositions(jointPos);
    return 0;
}

int RobotSystem::InverseKinematics(std::vector<std::pair<EEndEffectorType, Eigen::Isometry3d>>& eePoses, 
                                   Eigen::VectorXd& jointPos, bool ifUseWholeBody, bool ifApply) {
    std::shared_ptr<dart::dynamics::InverseKinematics> headIkPtr;
    std::shared_ptr<dart::dynamics::InverseKinematics> leftHandIkPtr;
    std::shared_ptr<dart::dynamics::InverseKinematics> rightHandIkPtr;
    for (auto eePose : eePoses) {
        if (eePose.first == HEAD) {
            headIkPtr = m_skel->getBodyNode("headEE")->getOrCreateIK();
            headIkPtr->getTarget()->setTransform(eePose.second);
            headIkPtr->setHierarchyLevel(1);
            headIkPtr->useWholeBody();
        } else if (eePose.first == LEFTHAND) {
            leftHandIkPtr = m_skel->getBodyNode("leftHandEE")->getOrCreateIK();
            leftHandIkPtr->getTarget()->setTransform(eePose.second);
            if (ifUseWholeBody) {
                leftHandIkPtr->useWholeBody();
            } else {
                leftHandIkPtr->useChain();
            }
        } else if (eePose.first == RIGHTHAND) {
            rightHandIkPtr = m_skel->getBodyNode("rightHandEE")->getOrCreateIK(); //set a Ik pointer
            rightHandIkPtr->getTarget()->setTransform(eePose.second);
            if (ifUseWholeBody) {
                rightHandIkPtr->useWholeBody();
            } else {
                rightHandIkPtr->useChain();
            }
        }
    }
    if (!m_skel->getIK(true)->findSolution(jointPos)) return -1;
    if (ifApply) m_skel->setPositions(jointPos);
    return 0;
}

Eigen::VectorXd RobotSystem::GetJointPos() const { return m_skel->getPositions(); }

Eigen::VectorXd RobotSystem::GetJointVel() const { return m_skel->getVelocities(); }

RobotState& RobotSystem::GetRobotState() { return m_robotState; }

RobotConfig& RobotSystem::GetRobotConfig() { return m_robotConfig; }

void RobotSystem::GetTcpOffset(EEndEffectorType type, Eigen::Isometry3d& offset) const { //需要抽象
    if (type == EEndEffectorType::LEFTHAND) offset = m_jointId.at("leftArmEEJoint")->getTransformFromParentBodyNode();
    if (type == EEndEffectorType::RIGHTHAND) offset = m_jointId.at("rightArmEEJoint")->getTransformFromParentBodyNode();
    return;
}

void RobotSystem::GetTcpPayload(EEndEffectorType type, Eigen::VectorXd& parameters) const { //需要抽象, 函数名需要修改
    if (type == EEndEffectorType::LEFTHAND) {
        parameters.resize(10);
        parameters(0) = m_linkId.at("leftHandEE")->getMass();
        parameters.segment(1, 3) = m_linkId.at("leftHandEE")->getLocalCOM();
        m_linkId.at("leftHandEE")
            ->getMomentOfInertia(parameters(4), parameters(5), parameters(6), parameters(7), parameters(8),
                                 parameters(9));
    }
    if (type == EEndEffectorType::RIGHTHAND) {
        parameters.resize(10);
        parameters(0) = m_linkId.at("rightHandEE")->getMass();
        parameters.segment(1, 3) = m_linkId.at("rightHandEE")->getLocalCOM();
        m_linkId.at("rightHandEE")
            ->getMomentOfInertia(parameters(4), parameters(5), parameters(6), parameters(7), parameters(8),
                                 parameters(9));
    }
}

std::map<std::string, double> RobotSystem::Vector2Map(const Eigen::VectorXd& vec) const {
    std::map<std::string, double> ret;

    for (std::map<std::string, dart::dynamics::JointPtr>::const_iterator it = m_jointId.cbegin();
         it != m_jointId.cend(); it++) {
        int joint_id = GetJointIdx(it->first);
        ret[it->first] = vec[joint_id];
    }

    return ret;
}

Eigen::VectorXd RobotSystem::Map2Vector(std::map<std::string, double> map) const {
    Eigen::VectorXd vec = Eigen::VectorXd::Zero(m_robotState.jointPositions.size());

    for (std::map<std::string, double>::const_iterator it = map.cbegin(); it != map.cend(); it++) {
        int joint_id = GetJointIdx(it->first);
        vec[joint_id] = it->second;
    }

    return vec;
}

Eigen::Isometry3d RobotSystem::GetLinkPose(const std::string& linkId) const {
    return m_linkId.at(linkId)->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
}

Eigen::Vector6d RobotSystem::GetLinkTwist(const std::string& linkId) const { //函数名可能需要修改
    Eigen::Vector6d twist;
    twist.head(3) =
        m_linkId.at(linkId)->getLinearVelocity(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    twist.tail(3) =
        m_linkId.at(linkId)->getAngularVelocity(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    return twist;
}

Eigen::Vector3d RobotSystem::GetLinkLinVel(const std::string& linkId) const {
    return m_linkId.at(linkId)->getLinearVelocity(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
}

Eigen::Vector3d RobotSystem::GetLinkAngVel(const std::string& linkId) const {
    return m_linkId.at(linkId)->getAngularVelocity(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotSystem::GetLinkJacobian(const std::string& linkId) const {
    return m_skel->getJacobian(m_linkId.at(linkId), Eigen::Vector3d::Zero(), dart::dynamics::Frame::World());
}

Eigen::Matrix<double, 6, 1> RobotSystem::GetLinkJacobianDotTimesQdot(const std::string& linkId) const {
    return m_skel->getJacobianClassicDeriv(m_linkId.at(linkId), Eigen::Vector3d::Zero(),
                                           dart::dynamics::Frame::World()) * GetJointVel();
}

Eigen::MatrixXd RobotSystem::GetMassMatrix() const { return m_skel->getMassMatrix(); }

Eigen::VectorXd RobotSystem::GetGravity() const { return m_skel->getGravityForces(); }

Eigen::VectorXd RobotSystem::GetCoriolis() const { return m_skel->getCoriolisForces(); }

// Eigen::VectorXd RobotSystem::GetFeedForwardTorque() {
//     return m_skel->getMassMatrix() * m_robotState.jointAccelerations + m_skel->getCoriolisForces() +
//            m_skel->getGravityForces();
//     ;
// }

int RobotSystem::GetJointIdx(const std::string& jointName) const {
    return m_jointId.at(jointName)->getIndexInSkeleton(0);
}

int RobotSystem::GetDof() const { return m_dof; }

dart::dynamics::SkeletonPtr RobotSystem::GetSkeleton() const { return m_skel; };
}  // namespace WorkSpaceAnalysis