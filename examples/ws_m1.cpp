#include <iostream>
#include <fstream>
#include <octomap/octomap.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/utils/utils.hpp>
#include <dart/simulation/World.hpp>
#include <dart/dynamics/SmartPointer.hpp>
#include <dart/collision/collision.hpp>
#include <Eigen/Dense>
#include <random>
#include <vector>
#include <dart/gui/osg/osg.hpp>
#include <osgGA/TrackballManipulator>
#include <chrono>
#include <filesystem> 
#include "SpaceSampler.h"
#include "Analyzer.h"
#include <robot/RobotSystem.h>
#include <ctime>
#include <csignal>
#include <CtrlC.h>
//#include <octovis/octovis.h>
// 修正类定义

class SaveLoadNewVisitPointCount{
public:
    SaveLoadNewVisitPointCount( const std::string& filename):  m_filename(filename) {}

    void saveNewVisitPointCount(int count) {
        std::ofstream outFile(m_filename);
        if (outFile.is_open()) {    
        outFile << count;
        outFile.close();
        std::cout << "NewVisitPointCount saved to " << m_filename << std::endl;
    } else {
        std::cerr << "Unable to open file for writing: " << m_filename << std::endl;
    }
    }

    int loadNewVisitPointCount() {
        std::ifstream inFile(m_filename);
        int count = 0;
        if (inFile.is_open()) {
            inFile >> count;
            inFile.close();
            std::cout << "NewVisitPointCount loaded from " << m_filename << std::endl;
        } else {
            std::cerr << "Unable to open file for reading: " << m_filename << std::endl;
        }
        return count;
        }
    private:
        std::string m_filename;
};

class CollisionDetector{
public:
    CollisionDetector(dart::dynamics::SkeletonPtr& m1, dart::simulation::WorldPtr& world, Eigen::VectorXd& jointTorqueLimits): m_m1(m1), m_world(world), m_jointTorqueLimits(jointTorqueLimits) {}

    bool checkCollision(){
        auto collisionDetector = m_world->getConstraintSolver()->getCollisionDetector(); // 检测自碰撞    
        auto collisionGroup = collisionDetector->createCollisionGroup(m_m1.get()); //创建一个碰撞组（CollisionGroup），包含机器人所有的碰撞几何体
        dart::collision::CollisionOption option;  //碰撞检测选项（如是否需要详细结果）
        dart::collision::CollisionResult result;  //碰撞检测结果（如发生碰撞的物体对、碰撞点等）。

        bool inCollision = collisionGroup->collide(option, &result);
        if (inCollision) return true; 
        else return false;
        }

    bool checkJointTorque() {
        // 逆动力学计算每个关节的力矩
        m_m1->computeInverseDynamics();
        Eigen::VectorXd jointForces = m_m1->getForces();
        // std::cout << "\n### current jointForces: \n" << jointForces.transpose() << std::endl;
        for (int i = 0; i < jointForces.size(); ++i) {
            if (jointForces[i] > m_jointTorqueLimits[i] || jointForces[i] < -m_jointTorqueLimits[i]) {
                // std::cout << "joint " << i << " torque invalid!\n" << std::endl;
                return true;
            }
        }
        return false;
    }

private:
    dart::dynamics::SkeletonPtr m_m1;
    dart::simulation::WorldPtr m_world; 
    Eigen::VectorXd m_jointTorqueLimits;
};

class RandomJointsConfigs{
public:
    RandomJointsConfigs(const dart::dynamics::SkeletonPtr& m1, const int& singlechaindofs, Eigen::VectorXd& config, const int& revoluteJointCount): 
    m_m1(m1), m_singlechaindofs(singlechaindofs), m_config(config), m_revoluteJointCount(revoluteJointCount) {}
    
    double generateRandom(double lower, double upper, double resolution) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(lower, upper);

    // 使用分辨率对随机数进行调整
    double randomValue = dis(gen);
    return std::round(randomValue / resolution) * resolution;
    }
    
    //返回所有旋转关节的索引
    std::vector<int> getrevoluteJointIndex(){ 
        std::vector<int> revoluteJointIndex;
        int jointCount = 0;

        for (size_t i = 0; i < m_m1->getNumJoints(); ++i) { // 遍历单链关节
            auto joint = m_m1->getJoint(i);
            if (jointCount >= m_singlechaindofs) break;
            if (auto revoluteJoint = dynamic_cast<dart::dynamics::RevoluteJoint*>(joint)){
                //revoluteJointCount++;
                revoluteJointIndex.push_back(i);
                std::cout << "revoluteJoint: " << m_m1->getJoint(i)->getName() << std::endl;
                jointCount++;
                std::cout << "jointCount: " << jointCount << std::endl;
            } // 如果关节是旋转关节
        }

        std::cout << "revoluteJointIndex: " << revoluteJointIndex.size() << std::endl;
        return revoluteJointIndex;
    }
    
    //对所有旋转关节进行随机采样，返回一组关节配置
    Eigen::VectorXd getConfig(std::vector<int> revoluteJointIndex){ 
        int index = 0;
        for (auto i : revoluteJointIndex) { // 遍历单链关节
            auto joint = m_m1->getJoint(i);
            if (auto revoluteJoint = dynamic_cast<dart::dynamics::RevoluteJoint*>(joint)) { // 如果关节是旋转关节
                //std::cout << "Sampling for joint: " << joint->getName() << std::endl;
                // 获取关节的上下限
                double lower = joint->getPositionLowerLimit(0);
                double upper = joint->getPositionUpperLimit(0);
                // 创建随机分布
                //std::uniform_real_distribution<> dis(lower, upper);
                double randomAngle = generateRandom(lower, upper, 0.02);
                m_config[index] = randomAngle;
                index++;
            }
        }

    //将左臂的7个关节角配置复制给右臂，使得两个臂对称
    m_config.segment(m_singlechaindofs, m_revoluteJointCount-m_singlechaindofs) = m_config.segment(m_singlechaindofs-(m_revoluteJointCount-m_singlechaindofs), m_revoluteJointCount-m_singlechaindofs);
    //std::cout << "config: " << config.transpose() << std::endl;
    return m_config;
}
private:
    dart::dynamics::SkeletonPtr m_m1;
    int m_singlechaindofs;
    Eigen::VectorXd m_config;
    int m_revoluteJointCount;
};

bool FK(octomap::OcTree& tree, dart::dynamics::SkeletonPtr m1, double ratio, std::vector<int> revoluteJointIndex, bool& IKflag, std::vector<double>& samplebound, dart::simulation::WorldPtr& world, Eigen::VectorXd& jointTorqueLimits, int& NewvisitPointCount, RandomJointsConfigs& randomJointsConfigs, int treeSize){
    int totalPointCount = 0;
    double constant = 2;
    //int treeSize = tree.size();

    auto start_time = std::chrono::steady_clock::now();
    auto interval_time = std::chrono::minutes(10);
    auto target_time = start_time + interval_time;
    while(true){
        auto current_time = std::chrono::steady_clock::now();
        if (current_time == target_time && static_cast<double>(NewvisitPointCount)/static_cast<double>(treeSize) == constant){ // 如果新访问的点数占总点数的比例不再增加，则停止循环
            std::cout << "No more new points" << std::endl;
            break;
        }
        Eigen::VectorXd config = randomJointsConfigs.getConfig(revoluteJointIndex);
        m1->setPositions(config);
        //std::cout << "set positions" << std::endl;
        //auto leftEndEffector = m1->getBodyNode("left_hand");
        auto rightEndEffector = m1->getBodyNode("rightHandEE");
        Eigen::VectorXd positions(3);
        //auto rh = rightEndEffector->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
        positions = rightEndEffector->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World()).translation(); // 只获取了右手末端执行器的位置，左手末端执行器的位置没有获取
        octomap::point3d point(positions[0], positions[1], positions[2]);

        if (point.x() > samplebound[0] && point.x() < samplebound[1] && point.y() > samplebound[2] && point.y() < samplebound[3] && point.z() > samplebound[4] && point.z() < samplebound[5]){ // 判断点是否在有效范围内
            octomap::OcTreeNode* node = tree.search(point);
            CollisionDetector CollisionDetector(m1, world, jointTorqueLimits);
            bool collisionFlag = CollisionDetector.checkCollision();
            bool jointTorqueFlag = CollisionDetector.checkJointTorque();
            if (node == nullptr && !collisionFlag && !jointTorqueFlag) {
                tree.updateNode(point, true);
                NewvisitPointCount++;
            }
            // octomap::point3d mirrorpoint = point; // 对称点, 对称点在y轴上对称,右手能到的点,左手也能到
            // mirrorpoint.y() = -point.y();
            // octomap::OcTreeNode* mirrornode = tree.search(mirrorpoint);
            // if (mirrornode == nullptr && !collisionFlag && !jointTorqueFlag) {
            //     tree.updateNode(mirrorpoint, true);
            //     NewvisitPointCount++;
            // }
        }
        totalPointCount++;
        //std::cout << "totalPointCount: " << totalPointCount << std::endl;
        std::cout << "NewvisitPointCount: " << NewvisitPointCount << " " << static_cast<double>(NewvisitPointCount)/static_cast<double>(treeSize)*100 << "%" << std::endl;
        if (totalPointCount%1000 == 0 && static_cast<double>(NewvisitPointCount)/static_cast<double>(totalPointCount) < ratio) {
            std::cout << "static_cast<double>(NewvisitPointCount)/static_cast<double>(totalPointCount) < ratio" << std::endl;
            IKflag = true;
            return true;
            }
        else if (totalPointCount%1000 == 0 && static_cast<double>(NewvisitPointCount)/static_cast<double>(treeSize) >= 0.80){
            IKflag = false;
            std::cout << "static_cast<double>(NewvisitPointCount)/static_cast<double>(treeSize): " << static_cast<double>(NewvisitPointCount)/static_cast<double>(treeSize) << std::endl;
            return false;
            }
        constant = static_cast<double>(NewvisitPointCount)/static_cast<double>(treeSize);
        target_time = start_time + interval_time;
    }
    return false;
}

dart::dynamics::SimpleFramePtr createPointCloudFrame() {
    auto pointCloudShape = ::std::make_shared<::dart::dynamics::PointCloudShape>();
    pointCloudShape->setPointShapeType(::dart::dynamics::PointCloudShape::BOX);
    auto pointCloudFrame = ::dart::dynamics::SimpleFrame::createShared(dart::dynamics::Frame::World());
    pointCloudFrame->setName("point cloud");
    pointCloudFrame->setShape(pointCloudShape);
    auto visualAspect = pointCloudFrame->createVisualAspect();
    visualAspect->setRGB(dart::Color::Blue());

    return pointCloudFrame;
}

class VisualizationWorldNode : public dart::gui::osg::WorldNode {
public:
    VisualizationWorldNode(const dart::simulation::WorldPtr& world) : dart::gui::osg::WorldNode(world), m_world(world) {
        auto pointCloudFrame = mWorld->getSimpleFrame("point cloud");
        mPointCloudShape = std::dynamic_pointer_cast<dart::dynamics::PointCloudShape>(pointCloudFrame->getShape());
        mPointCloudShape->setColorMode(dart::dynamics::PointCloudShape::BIND_PER_POINT);
        mPointCloudShape->setPointShapeType(dart::dynamics::PointCloudShape::BOX);
        mPointCloudVisualAspect = pointCloudFrame->getVisualAspect();

        assert(mPointCloudShape);
        mPointCloudVisualAspect->show();
    }
    ~VisualizationWorldNode() {}

    void customPreStep() override {
        // octomap::Pointcloud pointCloud;
        // auto numPoints = 500u;
        // pointCloud =
        //     generatePointCloudInBox(numPoints, Eigen::Vector3d::Constant(-0.5), Eigen::Vector3d::Constant(0.5));
        // // Update point cloud
        // mPointCloudShape->setPoints(pointCloud);
        // auto colors = generatePointCloudColors(pointCloud);
        // assert(pointCloud.size() == colors.size());
        // mPointCloudShape->setColors(colors);
    }

    std::shared_ptr<const dart::dynamics::PointCloudShape> getPointCloudShape() const { return mPointCloudShape; }

    dart::dynamics::VisualAspect* getVoxelGridVisualAspect() { return mPointCloudVisualAspect; }

private:
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> generatePointCloudColors( //这段代码的作用是根据点云（pointCloud）的高度（Z 值），为每个点生成一个颜色向量 c=(r,g,b,a)c=(r,g,b,a)，并返回所有点的颜色列表。
        const octomap::Pointcloud& pointCloud) {
        const auto& points = mPointCloudShape->getPoints();
        double minZ = std::numeric_limits<double>::max();
        double maxZ = std::numeric_limits<double>::min();
        for (const auto& point : points) {
            minZ = std::min(minZ, point.z());
            maxZ = std::max(maxZ, point.z());
        }
        double diffZ = std::max(std::abs(maxZ - minZ), std::numeric_limits<double>::min());

        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> colors;
        colors.reserve(pointCloud.size());
        for (const auto& point : pointCloud) {
            float r = (point.z() - static_cast<float>(minZ)) / static_cast<float>(diffZ);
            float g = 0.0f;
            float b = 1.f - r;
            r = dart::math::clip(r, 0.1f, 0.9f);
            g = dart::math::clip(g, 0.1f, 0.9f);//使用 dart::math::clip 将颜色值限制在 [0.1, 0.9] 范围内，避免颜色过暗或过亮。
            b = dart::math::clip(b, 0.1f, 0.9f);
            colors.emplace_back(Eigen::Vector4f(r, g, b, 0.75).cast<double>());
        }

        return colors;
    }

    dart::simulation::WorldPtr m_world;
    std::shared_ptr<dart::dynamics::PointCloudShape> mPointCloudShape;

    dart::dynamics::VisualAspect* mPointCloudVisualAspect;
};

int main() {
    std::string treefilePath = "/home/haitaoxu/workspaceanalysis/M1_full.bt";
    //std::string treefilePath = "/home/haitaoxu/workspaceanalysis/M1_full_load.bt";
    auto start_time = std::chrono::steady_clock::now();

    dart::simulation::WorldPtr world = dart::simulation::World::create();
    dart::utils::DartLoader urdf;
    dart::dynamics::SkeletonPtr m1 = urdf.parseSkeleton("/home/haitaoxu/workspaceanalysis/models/M1/M1_full.urdf");  // 加载 URDF 文件
    //dart::dynamics::SkeletonPtr m1 = urdf.parseSkeleton("/home/haitaoxu/workspaceanalysis/models/M1/M1_full_load.urdf");  // 加载 URDF 文件
    world->addSkeleton(m1);
    // dart::dynamics::SkeletonPtr ground = urdf.parseSkeleton("/home/haitaoxu/workspaceanalysis/models/Environment/ground_terrain.urdf");
    // world->addSkeleton(ground);

    enum ProgramProcessMode{
    ProgramProcessMode_Initial = 0, //
    ProgramProcessMode_Continue = 1,
    ProgramProcessMode_Visualize = 2,
    };
    ProgramProcessMode ProgramProcessMode = ProgramProcessMode_Visualize;


    Eigen::VectorXd jointTorqueLimits = Eigen::VectorXd(m1->getNumDofs()).setOnes(); // 可以手动修改关节力矩限制(修改时要注意joint数！)
    jointTorqueLimits << 373.52, 213.44, 213.44, 10.17, 94.19, 94.22, 50.44, 49.88, 18.69, 18.59, 10.17, 94.19, 94.22, 50.44, 49.88, 18.69, 18.59, 10.17;
    
    double resolution = 0.01;



    double x_min = -0.2, x_max = 2.0;
    double y_min, y_max;
    double z_min = 0.0, z_max = 2.0;

    auto rh = m1->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    Eigen::Vector3d eePos = rh.translation();
    y_min = eePos.y();
    y_max = eePos.y();
    // 设置负载物体中心位置的采样范围
    // 10kg
    // double x_min = 0.3;
    // double x_max = 1.2;
    // double z_min = 0.2;
    // double z_max = 1.5;
    // double y_min = -0.8;
    // double y_max = 0.8;
    // 25kg
    //  double x_min = 0.0;
    //  double x_max = 1.2;
    //  double z_min = 0.0;
    //  double z_max = 1.7;
    //  double y_min = 0.0;
    //  double y_max = 0.0;
    std::vector<double> samplebound = {x_min, x_max, y_min, y_max, z_min, z_max};

    //SpaceSampler sampler;
    std::vector<Eigen::Vector3d> samplePoints;
    auto SpaceSamplerPtr = std::make_shared<SpaceSampler>();
    //sampler.SpaceSampler::Discretize3DSpace(Eigen::Map<Eigen::Vector6d>(samplebound.data()), resolution, samplePoints); //这样才能得到正确的离散的所有点
    SpaceSamplerPtr->Discretize3DSpace(Eigen::Map<Eigen::Vector6d>(samplebound.data()), resolution, samplePoints); //这样才能得到正确的离散的所有点
    int samplePointsSize = samplePoints.size();
    std::cout << "samplePointsSize: " << samplePointsSize << std::endl;
    octomap::OcTree tree(0.01);

    Eigen::VectorXd config = Eigen::VectorXd::Zero(m1->getNumDofs()); //注意： getNumJoints() 返回的是所有关节的数量，而 getNumDofs() 返回的是自由度的数量
    
    int singlechaindofs = 11; // 假设单链的自由度数量
    int revoluteJointCount = m1->getNumDofs();
    RandomJointsConfigs randomJointsConfigs(m1, singlechaindofs, config, revoluteJointCount);
    std::vector<int> revoluteJointIndex = randomJointsConfigs.getrevoluteJointIndex();

    std::string filename = "/home/haitaoxu/workspaceanalysis/NewvisitPointCount_load.txt";
    SaveLoadNewVisitPointCount SaveLoadNewVisitPointCount(filename);
    int NewvisitPointCount = 0;

    if (ProgramProcessMode == ProgramProcessMode_Initial){
        std::cout << "ProgramProcessMode_Initial" << std::endl;

        std::cout << "FK" << std::endl; // 计算正运动学
        double ratio = 6/100;
        bool IKflag = false;
        IKflag = FK(tree, m1, ratio, revoluteJointIndex, IKflag, samplebound, world, jointTorqueLimits, NewvisitPointCount, randomJointsConfigs, samplePointsSize);
    }
    else if (ProgramProcessMode == ProgramProcessMode_Continue){
        std::cout << "ProgramProcessMode_Continue" << std::endl;

        NewvisitPointCount = SaveLoadNewVisitPointCount.loadNewVisitPointCount();
        if (std::filesystem::exists(treefilePath)){ // 如果树文件存在，则读取树文件
            tree.readBinary(treefilePath);
            
            std::ifstream ifs("/home/haitaoxu/workspaceanalysis/NewvisitPointCount_load.txt");
            if (ifs.is_open()) {
                std::string line;
                while (std::getline(ifs, line)) {
                    if (line.find("NewvisitPointCount=") != std::string::npos) {
                        NewvisitPointCount = std::stoi(line.substr(line.find('=') + 1));
                    } 
                }
                std::cout << "Restored state. NewvisitPointCount: " << NewvisitPointCount << std::endl;
            } else {
                std::cout << "No previous state found. Starting fresh.\n";
                NewvisitPointCount = 0;
            }

            std::cout << "FK" << std::endl; // 计算正运动学
            double ratio = 6/100;
        }
        else{
            std::cout << "treefilePath not exists" << std::endl;
        }
    }
    else if (ProgramProcessMode == ProgramProcessMode_Visualize){
        std::cout << "ProgramProcessMode_Visualize" << std::endl;

        NewvisitPointCount = SaveLoadNewVisitPointCount.loadNewVisitPointCount();
        if (std::filesystem::exists(treefilePath)){ // 如果树文件存在，则读取树文件
            tree.readBinary(treefilePath);
        }
        else{
            std::cout << "treefilePath not exists" << std::endl;
        }
    }

    auto pointCloudFrame = createPointCloudFrame(); //createPointCloudFrame 函数创建一个 SimpleFrame，用于表示点云在仿真环境中的位置和方向。
    auto pointCloudShape = std::dynamic_pointer_cast<dart::dynamics::PointCloudShape>(pointCloudFrame->getShape());

    octomap::Pointcloud pointCloud;
    pointCloud.clear();
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> colors; //这段代码定义了一个名为 colors 的 std::vector 容器，其元素类型为 Eigen::Vector4d，同时使用了 Eigen::aligned_allocator 来管理内存对齐。以下是详细解析。
    for (auto positions:samplePoints) {
        octomap::point3d point(positions[0], positions[1], positions[2]);
        octomap::OcTreeNode* node = tree.search(point);
        //std::cout << "pcl node->getOccupancy(): " << node->getOccupancy() << std::endl;
        if (node != nullptr) {
            if (node->getOccupancy() >= 0.5) {
            //std::cout << "it->getValue() == 1" << std::endl;
            //std::cout << "pcl node->getValue(): " << node->getValue() << std::endl;
            pointCloud.push_back(point.x(), point.y(), point.z());
            colors.push_back({0, 0, 1, 0.5});
            //std::cout << "pcl pointCloud.size(): " << pointCloud.size() << std::endl;
            }
        }
    }

    std::cout << "pcl pointCloud.size(): " << pointCloud.size() << std::endl;
    std::cout << "Space sampled point size:"<< samplePointsSize << std::endl;
    std::cout << "Valid Points in Space Ratio:"<< static_cast<double>(pointCloud.size())/static_cast<double>(samplePointsSize)*100 << "%" << std::endl;
    pointCloudShape->setVisualSize(resolution);

    // 添加工作区间的边界线
    //SpaceSampler sampler;
    std::vector<Eigen::Vector3d> boundaryPoints = SpaceSamplerPtr->get2dBoundaryPoints(resolution, samplebound);

    for (auto point : boundaryPoints) {
        pointCloud.push_back(point.x(), point.y(), point.z());
        colors.push_back({1, 0, 0, 0.5}); 
    }
    assert(pointCloud.size() == colors.size());
    pointCloudShape->setPoints(pointCloud);
    pointCloudShape->setColors(colors);
    world->addSimpleFrame(pointCloudFrame);

    // 最终可视化设置 -------------------------------------------------------------------------------------------------------
    
    // 可视化时skel变回零位
    m1->setPositions(Eigen::VectorXd(revoluteJointCount).setZero());
    m1->getJoint(0)->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());

    Eigen::Vector3d gravity(0.0, 0.0, 0.0);
    world->setGravity(gravity);
    world->setTimeStep(0.001);

    osg::ref_ptr<VisualizationWorldNode> node;
    node = new VisualizationWorldNode(world);

    osg::ref_ptr<dart::gui::osg::Viewer> viewer = new dart::gui::osg::Viewer();
    viewer->allowSimulation(false);
    viewer->addWorldNode(node);
    viewer->switchHeadlights(false);
    viewer->getLightSource(0)->getLight()->setPosition(::osg::Vec4(1.0, 0.2, 1.0, 0.0));
    viewer->setUpViewInWindow(0, 0, 1000, 900);
    // 视角切换为y方向看向x,z平面
    float eyeY = -5.0;
    // float xAxisOffset = (x_max - x_min) / 2.0;
    float xAxisOffset = 0.85;
    // float zAxisOffset = (z_max - z_min) / 2.0;
    float zAxisOffset = 0.9;
    ::osg::Vec3 up_xzView = ::osg::Vec3(0.0, 0.0, 1.0);

    // viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    // viewer->getCameraManipulator()->setHomePosition(::osg::Vec3(xAxisOffset, eyeY, zAxisOffset),::osg::Vec3(xAxisOffset, 0.0, zAxisOffset), up_xzView);
    // viewer->setCameraManipulator(viewer->getCameraManipulator());

    viewer->getCameraManipulator()->setHomePosition(::osg::Vec3(xAxisOffset, eyeY, zAxisOffset),::osg::Vec3(xAxisOffset, 0.0, zAxisOffset), up_xzView);
    viewer->setCameraManipulator(viewer->getCameraManipulator());

    // tree.writeBinary(treefilePath);
    // std::cout << "save tree.writeBinary success" << std::endl;
    // SaveLoadNewVisitPointCount.saveNewVisitPointCount(NewvisitPointCount);

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);  // 计算时间差
    std::cout << "Elapsed time: " << duration.count() << " milliseconds\n";  // 输出结果，单位是毫秒

    viewer->run();
    

    return 0;
}

