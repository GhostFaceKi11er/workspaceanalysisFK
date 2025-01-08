#pragma once

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include "utils/Types.hpp"

namespace WorkSpaceAnalysis {

class RobotSystem {
public:
    /// construct robot with urdf
    RobotSystem(const std::string& urdfFile);
    ~RobotSystem();

    /// set joint position limits to robot config
    void SetPositionLimits(const Eigen::VectorXd& posLowerLimits, const Eigen::VectorXd& posUpperLimits);

    /// set joint velocity limits to robot config
    void SetVelocityLimits(const Eigen::VectorXd& velLimits);

    /// set joint acceleration limtis to robot config
    void SetAccelerationLimits(const Eigen::VectorXd& accLimits);

    /// set joint jerk limits to robot config
    void SetJerkLimits(const Eigen::VectorXd& jerkLimits);

    /// set joint jerk limits to robot config
    void SetTorqueLimits(const Eigen::VectorXd& torqueLimits);

    /// set tcp offset for certain tcp
    void SetTcpOffset(EEndEffectorType type, const Eigen::Isometry3d& offset);

    /// set tcp payload for certain tcp (mass, localCOM, MomentofInertia)
    void SetTcpPayload(EEndEffectorType type, const Eigen::VectorXd& parameters);

    void GetTcpOffset(EEndEffectorType type, Eigen::Isometry3d& offset) const;

    void GetTcpPayload(EEndEffectorType type, Eigen::VectorXd& parameters) const;

    /// update robot system
    void UpdateSystem(const std::map<std::string, double>& jointPos, const std::map<std::string, double>& jointVel,
                      const Eigen::Vector3d& basePos = Eigen::Vector3d::Zero(),
                      const Eigen::Quaternion<double>& baseQuat = Eigen::Quaternion<double>::Identity(),
                      bool ifUpdateDynamics = true);
    /// update robot system
    void UpdateSystem(const Eigen::VectorXd& jointPos, const Eigen::VectorXd& jointVel,
                      const Eigen::Vector3d& basePos = Eigen::Vector3d::Zero(),
                      const Eigen::Quaternion<double>& baseQuat = Eigen::Quaternion<double>::Identity(),
                      bool ifUpdateDynamics = true);

    /**
     * @brief 单任务逆运动学
     * 
     * @param type 末端执行器类型
     * @param pose 目标位姿
     * @param jointPos 解算出的关节位置
     * @param ifUseWholeBody 是否使用全身关节参与逆运动学
     * @param ifApply 是否应用解得的关节位置到m_skel
     * @return int -1：失败，0：成功
     */
    int InverseKinematics(EEndEffectorType type, Eigen::Isometry3d& pose, Eigen::VectorXd& jointPos,
                          bool ifUseWholeBody, bool ifApply = false);

    /**
     * @brief 多任务逆运动学
     * 
     * @param eePoses 容器，存储末端执行器类型以及对应的目标位姿
     * @param jointPos 解算出的关节位置
     * @param ifUseWholeBody 是否使用全身关节参与逆运动学
     * @param ifApply 是否应用解得的关节位置到m_skel
     * @return int -1：失败，0：成功
     */
    int InverseKinematics(std::vector<std::pair<EEndEffectorType, Eigen::Isometry3d>>& eePoses,
                          Eigen::VectorXd& jointPos, bool ifUseWholeBody, bool ifApply = false);

    Eigen::VectorXd GetJointPos() const;

    Eigen::VectorXd GetJointVel() const;

    RobotState& GetRobotState();

    RobotConfig& GetRobotConfig();

    std::map<std::string, double> Vector2Map(const Eigen::VectorXd& vec) const;

    Eigen::VectorXd Map2Vector(std::map<std::string, double> map) const;

    Eigen::Isometry3d GetLinkPose(const std::string& linkId) const;

    Eigen::Vector6d GetLinkTwist(const std::string& linkId) const;

    Eigen::Vector3d GetLinkLinVel(const std::string& linkId) const;

    Eigen::Vector3d GetLinkAngVel(const std::string& linkId) const;

    Eigen::Matrix<double, 6, Eigen::Dynamic> GetLinkJacobian(const std::string& linkId) const;

    Eigen::Matrix<double, 6, 1> GetLinkJacobianDotTimesQdot(const std::string& linkId) const;

    //--------------------dynamics------------------------
    Eigen::MatrixXd GetMassMatrix() const;

    Eigen::VectorXd GetGravity() const;

    Eigen::VectorXd GetCoriolis() const;

    int GetJointIdx(const std::string& joint_name) const;

    int GetDof() const;

    dart::dynamics::SkeletonPtr GetSkeleton() const;

    /// Robot state
    RobotState m_robotState;

    // Robot config(limits)
    RobotConfig m_robotConfig;

private:
    void ConfigRobot();

    int m_dof;

    dart::dynamics::SkeletonPtr m_skel;
    std::string m_urdfFile;
    /// Map of joint name and dart's JointPtr
    std::map<std::string, dart::dynamics::JointPtr> m_jointId;

    /// Map of link name and dart's BodyNodePtr
    std::map<std::string, dart::dynamics::BodyNodePtr> m_linkId;
};
}  // namespace WorkSpaceAnalysis