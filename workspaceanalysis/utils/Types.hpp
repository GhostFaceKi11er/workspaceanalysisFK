#pragma once

namespace WorkSpaceAnalysis {

constexpr double EPS3 = 1E-3;
constexpr double EPS4 = 1E-4;
constexpr double EPS5 = 1E-5;
constexpr double EPS6 = 1E-6;
constexpr double EPS9 = 1E-9;
constexpr double EPS12 = 1E-12;

enum EMoveType { MOVEJ = 0, MOVEL };

enum EMoveMode { SYNCHRONOUS = 0, ASYNCHRONOUS };

enum EControlPolicy { FULLBODY = 0, SINGLEARM };

enum EEndEffectorType {
    HEAD = 0,
    LEFTHAND,
    RIGHTHAND,

    NUM_OF_ENDEFFECTORS
};

class RobotState {
public:
    void Resize(int dof) {
        jointPositions.resize(dof);
        jointVelocities.resize(dof);
        jointAccelerations.resize(dof);
    }
    Eigen::VectorXd jointPositions;
    Eigen::VectorXd jointVelocities;
    Eigen::VectorXd jointAccelerations;
};

class RobotConfig {
public:
    void Resize(int dof) {
        jointPosLowerLimits.resize(dof);
        jointPosUpperLimits.resize(dof);
        jointVelLimits.resize(dof);
        jointAccLimits.resize(dof);
        jointJerkLimits.resize(dof);
        jointTorqueLimits.resize(dof);
    }
    Eigen::VectorXd jointPosLowerLimits;
    Eigen::VectorXd jointPosUpperLimits;
    Eigen::VectorXd jointVelLimits;
    Eigen::VectorXd jointAccLimits;
    Eigen::VectorXd jointJerkLimits;
    Eigen::VectorXd jointTorqueLimits;

    Eigen::Vector2d cartVelLimits[EEndEffectorType::NUM_OF_ENDEFFECTORS];
    Eigen::Vector2d cartAccLimits[EEndEffectorType::NUM_OF_ENDEFFECTORS];
    Eigen::Vector2d cartJerkLimits[EEndEffectorType::NUM_OF_ENDEFFECTORS];
};

class MoveJPlanInfo {
public:
    int dof;
    Eigen::VectorXd jointPosInit;
    Eigen::VectorXd jointPosFinal;
    Eigen::VectorXd jointVelLimits;
    Eigen::VectorXd jointAccLimits;
    Eigen::VectorXd jointJerkLimits;
};

class MoveLPlanInfo {
public:
    Eigen::Vector3d cartPosInit;
    Eigen::Vector3d cartPosFinal;
    Eigen::AngleAxisd cartOriInit;  // 轴角表示
    Eigen::AngleAxisd cartOriFinal;

    Eigen::Vector2d cartVelLimits;
    Eigen::Vector2d cartAccLimits;
    Eigen::Vector2d cartJerkLimits;
};

class JointMotion {
public:
    void Resize(int dof) {
        jointPos.resize(dof);
        jointVel.resize(dof);
        jointAcc.resize(dof);
    }
    Eigen::VectorXd jointPos;
    Eigen::VectorXd jointVel;
    Eigen::VectorXd jointAcc;
};

class CartMotion {
public:
    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
    Eigen::Vector3d linVel;
    Eigen::Vector3d oriVel;
    Eigen::Vector3d linAcc;
    Eigen::Vector3d oriAcc;
};

class JointPlanInfo {
public:
    JointPlanInfo(const Eigen::VectorXd &jointPosInit, const Eigen::VectorXd &jointPosFinal, double velPer,
                  double accPer)
        : m_jointPosInit(jointPosInit), m_jointPosFinal(jointPosFinal), m_velPer(velPer), m_accPer(accPer) {}
    Eigen::VectorXd m_jointPosInit;
    Eigen::VectorXd m_jointPosFinal;

    double m_velPer;
    double m_accPer;
};

class EndEffectorPlanInfo {
public:
    EndEffectorPlanInfo(EEndEffectorType type, const Eigen::Isometry3d &cartPoseInit,
                        const Eigen::Isometry3d &cartPoseFinal, double velPer, double accPer)
        : m_type(type),
          m_cartPoseInit(cartPoseInit),
          m_cartPoseFinal(cartPoseFinal),
          m_velPer(velPer),
          m_accPer(accPer) {}
    EEndEffectorType m_type;

    Eigen::Isometry3d m_cartPoseInit;
    Eigen::Isometry3d m_cartPoseFinal;

    double m_velPer;
    double m_accPer;
};

typedef std::vector<EndEffectorPlanInfo> EndEffectorPlanInfos;

inline void AvoidQuatJump(const Eigen::Quaternion<double> &des_ori, Eigen::Quaternion<double> &act_ori) {
    Eigen::Quaternion<double> ori_diff1; //这段代码定义了一个内联函数 AvoidQuatJump，其目的是通过调整四元数（Quaternion）的符号来
    Eigen::Quaternion<double> ori_diff2; //避免由于四元数表达同一旋转的不同方式而导致的“跳变”（Jump）。这是一个常见的问题，因为四元
                                         //数 (q)(q) 和其相反数 (−q)(−q) 表示的是相同的旋转，但在某些优化或插值问题中，这种跳变可能会引发不必要的偏差或不连续性。
    ori_diff1.w() = des_ori.w() - act_ori.w();
    ori_diff1.x() = des_ori.x() - act_ori.x();
    ori_diff1.y() = des_ori.y() - act_ori.y();
    ori_diff1.z() = des_ori.z() - act_ori.z();

    ori_diff2.w() = des_ori.w() + act_ori.w();
    ori_diff2.x() = des_ori.x() + act_ori.x();
    ori_diff2.y() = des_ori.y() + act_ori.y();
    ori_diff2.z() = des_ori.z() + act_ori.z();

    if (ori_diff1.squaredNorm() > ori_diff2.squaredNorm()) {
        act_ori.w() = -act_ori.w();
        act_ori.x() = -act_ori.x();
        act_ori.y() = -act_ori.y();
        act_ori.z() = -act_ori.z();
    } else
        act_ori = act_ori;
}

inline Eigen::Vector3d QuatToExp(const Eigen::Quaternion<double> &quat) { //这段代码定义了一个内联函数 QuatToExp，用于将四元数（Quaternion）转换为旋转矢量的指数坐标形式（Exponential Coordinates，也称为轴角表示）。
    Eigen::Vector3d img_vec(quat.x(), quat.y(), quat.z());
    double w(quat.w());
    double theta(2.0 *
                 std::asin(std::sqrt(img_vec[0] * img_vec[0] + img_vec[1] * img_vec[1] + img_vec[2] * img_vec[2])));
    if (theta < 0.0001) {
        return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d ret = img_vec / std::sin(theta / 2.);
    return ret * theta;
}

inline Eigen::MatrixXd vStack(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b) { //这段代码定义了一个名为 vStack 的内联函数，作用是将两个矩阵 a 和 b 按垂直方向（纵向）拼接起来，形成一个新的矩阵
    assert(a.cols() == b.cols());
    Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
    ab << a, b;
    return ab;
}

}  // namespace WorkSpaceAnalysis