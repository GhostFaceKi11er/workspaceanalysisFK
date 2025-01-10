#include "Analyzer.h"
#include <csignal>
#include <fstream>
#include <CtrlC.h>

namespace WorkSpaceAnalysis { //这段代码是一个命名空间 WorkSpaceAnalysis 中的构造函数 WorkspaceAnalyzer::WorkspaceAnalyzer 的实现，用于初始化一个名为 WorkspaceAnalyzer 的类的对象。以下是详细解析。

// 定义静态变量
std::vector<Eigen::Vector3d> WorkspaceAnalyzer::m_validCartPoints;
Eigen::VectorXd WorkspaceAnalyzer::m_currentJointConfiguration;
Eigen::Vector3d eePos;
//auto rh = m_robot->GetSkeleton()->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());

WorkspaceAnalyzer::WorkspaceAnalyzer(const std::string& urdfFile) {
    m_robot = std::make_shared<RobotSystem>(urdfFile);
    m_sampler = std::make_shared<SpaceSampler>();
    m_checker = std::make_shared<Checker>(m_robot);
}

WorkspaceAnalyzer::~WorkspaceAnalyzer() {}

std::string WorkspaceAnalyzer::getCurrentDateTime(){
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);

    std::ostringstream oss;
    oss << std::put_time(localTime, "%Y%m%d%H%M%S");
    return oss.str();
}

// void WorkspaceAnalyzer::signalHandler(int signal) {
//     if (signal == SIGINT) {
//         //CtrlC::interrupted = true;
//         std::cout << "Caught signal, exiting program..." << std::endl;
//         std::cout << "end-effctor position when exit program:" << eePos << std::endl;
//         //CtrlC::saveState(CtrlC::lastCompletedStep, CtrlC::lastAngleStep);
//         std::string filename = "valid_cart_points.csv";
//         WorkspaceAnalyzer::SaveValidPointsToCSV(filename);
//         WorkspaceAnalyzer::saveCurrentJointConfiguration();
//         exit(0);
//     }
// }

bool WorkspaceAnalyzer::loadValidCartPoints() {
    std::ifstream ifs("valid_cart_points.csv");
    if (ifs.is_open()) {
        std::string line;
        while (std::getline(ifs, line)) {
            std::stringstream ss(line);
            std::string item;
            Eigen::Vector3d point;
            int index = 0;
            while (std::getline(ss, item, ',')) {
                if (index < 3) {
                    point[index] = std::stod(item);
                    index++;
                }
            }
            m_validCartPoints.push_back(point);
        }
        ifs.close();
        return true;
    } else {
        std::cerr << "No previous valid cart points found.\n";
        return false;
    }
}

void WorkspaceAnalyzer::saveCurrentJointConfiguration() {
    std::ofstream ofs("config.txt");
    if (ofs.is_open()) {
        ofs << m_currentJointConfiguration.transpose() << std::endl;
        ofs.close();
    } else {
        std::cerr << "Failed to open file for saving joint configuration.\n";
    }
}

void WorkspaceAnalyzer::loadCurrentJointConfiguration() {
    std::ifstream ifs("config.txt");
    if (ifs.is_open()) {
        std::vector<double> values;
        double value;
        while (ifs >> value) {
            values.push_back(value);
        }
        ifs.close();
        m_currentJointConfiguration = Eigen::VectorXd::Map(values.data(), values.size());
    } else {
        std::cerr << "Failed to open file for loading joint configuration.\n";
    }
}

void WorkspaceAnalyzer::SaveValidPointsToCSV(std::string& filename) {
    std::ofstream ofs(filename, std::ios::out | std::ios::trunc);
    if (ofs.is_open()) {
        for (const auto& point : m_validCartPoints) {
            ofs << point.x() << "," << point.y() << "," << point.z() << std::endl;
        }
        ofs.close();
    } else {
        std::cerr << "Failed to open CSV file for saving valid cart points.\n";
    }
}


void WorkspaceAnalyzer::AnalyzeCartSpace(EEndEffectorType& endEffectorType, Eigen::Vector6d posBoundaries,
                                         double posResolution, Eigen::Matrix3d initPose,
                                         std::pair<std::string, Eigen::Vector2d> angleBoundaries,
                                         double angleResolution) {
    
    std::vector<Eigen::Vector3d> sampledPositions;
    m_sampler->Discretize3DSpace(posBoundaries, posResolution, sampledPositions); //离散化距离
    std::cout << "--- Finished discretizing cart space,  " << sampledPositions.size() << " points generated ---" << std::endl;

    // 离散化角度，按一正一负的顺序，有利于减少后续角度循环的次数
    std::vector<double> sampledAngles;
    m_sampler->DiscretizeSingleAngleSpace(0, 1.57, 0.1, sampledAngles);
    std::cout << "--- Finished discretizing angle,  " << sampledAngles.size() << " angles generated ---" << std::endl;

    Eigen::Vector3d axis;
    if (angleBoundaries.first == "x") {
        axis = Eigen::Vector3d::UnitX();
    } else if (angleBoundaries.first == "y") {
        axis = Eigen::Vector3d::UnitY();
    } else if (angleBoundaries.first == "z") {
        axis = Eigen::Vector3d::UnitZ();
    } else {
        std::cout << "Analyzer.AnalyzeCartSpace: Invalid axis specified, please specify x, y or z!" << std::endl;
        return;
    }

    Eigen::Isometry3d rightHandTf;
    Eigen::Isometry3d leftHandTf;
    Eigen::VectorXd jointPos;
    
    bool bTorque;
    int countvalidCartPoints = 0;
    int count = 0;
    int countangle = 0;

    for (auto point:sampledPositions) {
        octomap::point3d m_point(point[0], point[1], point[2]);
        std::cout << "count/sampledPositions:" << count << "/" << sampledPositions.size()<< std::endl;
        std::cout << "countvalidCartPoints/sampledPositions:" << countvalidCartPoints << "/" << sampledPositions.size()<< std::endl;
        if (CtrlC::tree.search(m_point) == nullptr){
            rightHandTf.translation() = point;
            Eigen::Vector3d mirrorPoint = {point[0], -point[1], point[2]};
            leftHandTf.translation() = mirrorPoint;
            rightHandTf.translation()[1] = -0.15;
            leftHandTf.translation()[1] = 0.15;
        
            if (countangle==sampledAngles.size()) countangle = 0;
            
            for (countangle; countangle<sampledAngles.size(); ++countangle) {
                auto angle = sampledAngles[countangle];
                
                // 最新手部坐标系y轴均朝内，所以将离散的角度加到y轴旋转
                rightHandTf.linear() *= Eigen::AngleAxisd(angle, axis).toRotationMatrix();
                leftHandTf.linear() = rightHandTf.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

                std::vector<std::pair<WorkSpaceAnalysis::EEndEffectorType, Eigen::Isometry3d>> eePoses = { 
                {WorkSpaceAnalysis::EEndEffectorType::RIGHTHAND, rightHandTf}, {WorkSpaceAnalysis::EEndEffectorType::LEFTHAND, leftHandTf}};//对两个手的末端执行器都求逆解
                if (m_robot->InverseKinematics(eePoses, jointPos, true, true) == 0){
                    bTorque = m_checker->CheckJointTorque();
                    if (bTorque) {
                        CtrlC::tree.updateNode(m_point, true);
                        countvalidCartPoints++;
                        CtrlC::NewvisitPointCount++;
                        std::cout << "IK countvalidCartPoints: " << countvalidCartPoints << "  CtrlC::NewvisitPointCount: " << CtrlC::NewvisitPointCount << std::endl;
                        countangle = 0;
                        break;
                }
            }
        }
    }
    count++;
    }
}


void WorkspaceAnalyzer::AnalyzeCartSpaceWithLoad(EEndEffectorType& endEffectorType, Eigen::Vector6d posBoundaries,
                                                 double posResolution, Eigen::Matrix3d initPose,
                                                 std::pair<std::string, Eigen::Vector2d> angleBoundaries,
                                                 double angleResolution) {
    m_validPointsWithLoad.clear();

    std::vector<Eigen::Vector3d> sampledPositions;
    m_sampler->Discretize3DSpace(posBoundaries, posResolution, sampledPositions); //离散点存储在sampledPositions
    std::cout << "--- Finished discretizing cart space,  " << sampledPositions.size() << " points generated ---"
              << std::endl;

    // 离散化角度，按一正一负的顺序，有利于减少后续角度循环的次数
    std::vector<double> sampledAngles;
    m_sampler->DiscretizeSingleAngleSpace(0, 1.57, 0.1, sampledAngles);
    std::cout << "--- Finished discretizing angle,  " << sampledAngles.size() << " angles generated ---" << std::endl;

    // TEST: 只给零姿态
    //  sampledAngles = {0};

    Eigen::Vector3d axis;
    if (angleBoundaries.first == "x") {
        axis = Eigen::Vector3d::UnitX();
    } else if (angleBoundaries.first == "y") { //y +-1.57
        axis = Eigen::Vector3d::UnitY();
    } else if (angleBoundaries.first == "z") {
        axis = Eigen::Vector3d::UnitZ();
    } else {
        std::cout << "Analyzer.AnalyzeCartSpace: Invalid axis specified, please specify x, y or z!" << std::endl;
        return;
    }

    Eigen::Isometry3d rightHandTf;
    Eigen::Isometry3d leftHandTf;
    Eigen::VectorXd jointPos;

    bool bCollision;
    bool bTorque;
    bool bValid;
    int countvalidCartPoints;
    // TEST: 只对一个点/角度进行检查有效性
    //  sampledPositions = {Eigen::Vector3d(0.25, 0.0, 1.9)};
    //  sampledAngles = {-1.5};

    // 零位、下蹲初始姿态
    Eigen::VectorXd zeroJointPos = Eigen::VectorXd::Zero(m_robot->GetSkeleton()->getNumDofs());
    Eigen::VectorXd crouchJointPos = Eigen::VectorXd::Zero(m_robot->GetSkeleton()->getNumDofs());
    crouchJointPos[0] = M_PI / 6;
    crouchJointPos[1] = -M_PI / 6;

    for (auto point : sampledPositions) { //sampledPositions:y=0
        bValid = false;

        // 给定位置，左右手相距0.3, 和sampledPositions不在同一个xz平面
        rightHandTf.translation() = point;
        rightHandTf.translation()[1] -= 0.15;
        leftHandTf.translation() = point;
        leftHandTf.translation()[1] += 0.15; 

        for (auto angle : sampledAngles) {
            rightHandTf.linear() = initPose;
            // 最新手部坐标系y轴均朝内，所以将离散的角度加到右手初始位姿y轴旋转
            rightHandTf.linear() *= Eigen::AngleAxisd(angle, axis).toRotationMatrix(); //axis=y

            // 左手的姿态对称，即恒等于右手的姿态绕x轴旋转pi
            leftHandTf.linear() =
                rightHandTf.linear() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

            std::vector<std::pair<EEndEffectorType, Eigen::Isometry3d>> eePoses = {
                {EEndEffectorType::RIGHTHAND, rightHandTf}, {EEndEffectorType::LEFTHAND, leftHandTf}}; //对两个手的末端执行器都求逆解

            std::vector<Eigen::VectorXd> ikSolutions;

            // 对两种初始位姿进行singlearm、fullbody逆解，不进行apply；如果有解则存储，后续进行检查
            // @零位初始姿态
            m_robot->GetSkeleton()->setPositions(zeroJointPos);
            if (m_robot->InverseKinematics(eePoses, jointPos, false, false) == 0) {
                ikSolutions.push_back(jointPos);
            }
            if (m_robot->InverseKinematics(eePoses, jointPos, true, false) == 0) {
                ikSolutions.push_back(jointPos);
            }
            // @下蹲初始姿态
            m_robot->GetSkeleton()->setPositions(crouchJointPos);
            if (m_robot->InverseKinematics(eePoses, jointPos, false, false) == 0) {
                ikSolutions.push_back(jointPos);
            }
            if (m_robot->InverseKinematics(eePoses, jointPos, true, false) == 0) {
                ikSolutions.push_back(jointPos);
            }

            // 遍历所有逆解得到的关节位姿并在skel中set，检查力矩和碰撞
            // std::cout<<"ik solution size: "<<ikSolutions.size()<<std::endl;
            if (ikSolutions.size() != 0) {
                for (auto solution : ikSolutions) {
                    m_robot->GetSkeleton()->setPositions(solution);

                    // 检查力矩和碰撞
                    // bCollision = m_checker->CheckCollision();
                    bTorque = m_checker->CheckJointTorque();

                    // test:先不检查碰撞或力矩
                    bCollision = false;
                    // bTorque = true;

                    if ((!bCollision) && bTorque) {
                        m_validPointsWithLoad.push_back(point);
                        countvalidCartPoints++;
                        bValid = true;
                        // std::cout << point << " valid!" << std::endl;
                        break;
                    }
                }
            }
            if (bValid) { // 如果存在有效解，则跳出角度循环，对下一个点进行检查
                break;
            }
        }
    }

    std::cout << "--- Finished checking joint torque with load: " << m_validPointsWithLoad.size()
              << " points valid  ---" << std::endl;
}

void WorkspaceAnalyzer::GetValidCartPoints(std::vector<Eigen::Vector3d>& validPoints) const {
    validPoints = m_validCartPoints;
}

void WorkspaceAnalyzer::GetValidCartPointsWithLoad(std::vector<Eigen::Vector3d>& validPointsWithLoad) const {
    validPointsWithLoad = m_validPointsWithLoad;
}
}  // namespace WorkSpaceAnalysis