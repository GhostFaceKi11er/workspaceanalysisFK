#include "Checker.h"
namespace WorkSpaceAnalysis {
Checker::Checker(std::shared_ptr<RobotSystem> robot)
    : m_robot(robot), m_jointTorqueLimits(robot->GetRobotConfig().jointTorqueLimits) {
    m_skel = m_robot->GetSkeleton();
    m_skel->enableSelfCollisionCheck();
    m_skel->disableAdjacentBodyCheck();
    m_constraintSolverPtr = new dart::constraint::BoxedLcpConstraintSolver();
    m_constraintSolverPtr->addSkeleton(m_skel);
    m_fclCollisionDetector = dart::collision::FCLCollisionDetector::create();
    m_constraintSolverPtr->setCollisionDetector(m_fclCollisionDetector);
    m_group = m_constraintSolverPtr->getCollisionGroup();
    m_option = m_constraintSolverPtr->getCollisionOption();
    m_result = m_constraintSolverPtr->getLastCollisionResult();

    m_option.maxNumContacts = 1;
}
Checker::~Checker() { delete m_constraintSolverPtr; }

bool Checker::CheckCollision() {
    // bool ifCollide = m_group->collide(m_option, &m_result);
    // if (ifCollide) {
    //     auto collideBodyNodes = m_result.getCollidingBodyNodes();
    //     for (auto bodynode : collideBodyNodes) {
    //         std::cout << bodynode->getName() << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // return ifCollide;
    return m_group->collide(m_option);
}

bool Checker::CheckJointTorque() {
    // 逆动力学计算每个关节的力矩
    m_skel->computeInverseDynamics();
    Eigen::VectorXd jointForces = m_skel->getForces();
    // std::cout << "\n### current jointForces: \n" << jointForces.transpose() << std::endl;
    for (int i = 0; i < jointForces.size(); ++i) {
        if (jointForces[i] > m_jointTorqueLimits[i] || jointForces[i] < -m_jointTorqueLimits[i]) {
            // std::cout << "joint " << i << " torque invalid!\n" << std::endl;
            return false;
        }
    }
    return true;
}

}  // namespace WorkSpaceAnalysis