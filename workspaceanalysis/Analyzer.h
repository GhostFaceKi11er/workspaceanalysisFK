#pragma once

#include "robot/RobotSystem.h"
#include "SpaceSampler.h"
#include "Checker.h"

namespace WorkSpaceAnalysis {

class WorkspaceAnalyzer {
public:
    WorkspaceAnalyzer(const std::string& urdfFile);

    ~WorkspaceAnalyzer();

    //static void signalHandler(int signal);

    void AnalyzeCartSpace(EEndEffectorType& endEffectorType, Eigen::Vector6d posBoundaries, double posResolution,
                          Eigen::Matrix3d initPose, std::pair<std::string, Eigen::Vector2d> angleBoundaries,
                          double angleResolution);

    void AnalyzeCartSpaceWithLoad(EEndEffectorType& endEffectorType, Eigen::Vector6d posBoundaries,
                                  double posResolution, Eigen::Matrix3d initPose,
                                  std::pair<std::string, Eigen::Vector2d> angleBoundaries, double angleResolution);

    void GetValidCartPoints(std::vector<Eigen::Vector3d>& validPoints) const;

    void GetValidCartPointsWithLoad(std::vector<Eigen::Vector3d>& validPointsWithLoad) const;

    std::string getCurrentDateTime();

    static void SaveValidPointsToCSV(std::string&);

    void saveValidCartPoints();

    //static void signalHandler(int signal);

    bool loadValidCartPoints();

    void appendValidCartPointsToCSV(const std::string& filename);

    std::shared_ptr<RobotSystem> m_robot;

    int countsampledPositions;

    int countangle;

    static void outputCurrentJointConfiguration();

    static void saveCurrentJointConfiguration();
    void loadCurrentJointConfiguration();

private:
    std::shared_ptr<SpaceSampler> m_sampler;
    std::shared_ptr<Checker> m_checker;

    static std::vector<Eigen::Vector3d> m_validCartPoints;
    std::vector<Eigen::Vector3d> m_validPointsWithLoad;
    static Eigen::VectorXd m_currentJointConfiguration; // 假设这是当前关节配置
};
}  // namespace WorkSpaceAnalysis