#pragma once //pragma once 是一种用于防止头文件被重复包含的编译器指令。它通过确保头文件在一个编译单元中只被处理一次，解决了头文件重复包含可能引发的问题，例如符号重复定义。
#include <dart/dart.hpp>
#include "robot/RobotSystem.h"

namespace WorkSpaceAnalysis {

class Checker {
public:
    Checker(std::shared_ptr<RobotSystem> robot);

    ~Checker();

    /**
     * @brief 检查当前m_skel是否发生碰撞
     * 
     * @return true 发生碰撞
     * @return false 无碰撞
     */
    bool CheckCollision();

    /**
     * @brief 检查关节力矩是否超出限制
     * 
     * @return true 关节力矩合法，未超出限制
     * @return false 关节力矩超出限制
     */
    bool CheckJointTorque();

private:
    std::shared_ptr<RobotSystem>& m_robot;
    dart::dynamics::SkeletonPtr m_skel;
    dart::constraint::BoxedLcpConstraintSolver* m_constraintSolverPtr; //是 DART 动力学仿真库中的一种约束求解器（Constraint Solver），用于解决物理仿真中的线性互补问题（Linear Complementarity Problem, LCP）。它专门处理带有边界条件的约束问题，适用于碰撞检测、接触力计算和关节约束等场景。
    dart::collision::FCLCollisionDetectorPtr m_fclCollisionDetector; //Flexible Collision Library
    dart::collision::CollisionGroupPtr m_group; //碰撞组（Collision Group）是碰撞检测模块中用于组织和管理一组几何形状（如机器人、物体的形状帧等）的对象。碰撞检测通常在两个碰撞组之间进行，检查组内或组间的碰撞关系。
    dart::collision::CollisionOption m_option; //bool enableContact, size_t maxNumContacts, bool binaryCheck
    dart::collision::CollisionResult m_result; //用于存储碰撞检测的结果，包括是否发生碰撞、接触点等信息。

    Eigen::VectorXd& m_jointTorqueLimits;
};
}  // namespace WorkSpaceAnalysis