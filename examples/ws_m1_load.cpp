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

//检测碰撞，力矩是否超限
class Detector{
public:
    Detector(dart::dynamics::SkeletonPtr& skel, dart::simulation::WorldPtr& world, Eigen::VectorXd& jointTorqueLimits): m_skel(skel), m_world(world), m_jointTorqueLimits(jointTorqueLimits) {}
    
    // 检测自碰撞 
    bool checkCollision(){
        auto Detector = m_world->getConstraintSolver()->getCollisionDetector();    
        auto collisionGroup = Detector->createCollisionGroup(m_skel.get()); //创建一个碰撞组（CollisionGroup），包含机器人所有的碰撞几何体
        dart::collision::CollisionOption option;  //碰撞检测选项（如是否需要详细结果）
        dart::collision::CollisionResult result;  //碰撞检测结果（如发生碰撞的物体对、碰撞点等）。

        bool inCollision = collisionGroup->collide(option, &result);
        if (inCollision) return true; 
        else return false;
        }

    // 检测关节力矩是否超限
    bool checkJointTorque() {
        // 逆动力学计算每个关节的力矩
        m_skel->computeInverseDynamics();
        Eigen::VectorXd jointForces = m_skel->getForces();
        for (int i = 0; i < jointForces.size(); ++i) {
            if (jointForces[i] > m_jointTorqueLimits[i] || jointForces[i] < -m_jointTorqueLimits[i]) {
                return true;
            }
        }
        return false;
    }

private:
    dart::dynamics::SkeletonPtr m_skel;
    dart::simulation::WorldPtr m_world; 
    Eigen::VectorXd m_jointTorqueLimits;
};

//关节角随机采样
class RandomJointsConfigs{
public:
    RandomJointsConfigs(const dart::dynamics::SkeletonPtr& skel, const int& singlechaindofs, Eigen::VectorXd& config, const int& revoluteJointCount): 
    m_skel(skel), m_singlechaindofs(singlechaindofs), m_config(config), m_revoluteJointCount(revoluteJointCount) {}
    
    //对当前关节在上限，下限内根据分辨率随机采样
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
        for (size_t i = 0; i < m_skel->getNumJoints(); ++i) { // 遍历单链关节
            auto joint = m_skel->getJoint(i);
            if (jointCount >= m_singlechaindofs) break;

            // 如果关节是旋转关节
            if (auto revoluteJoint = dynamic_cast<dart::dynamics::RevoluteJoint*>(joint)){
                revoluteJointIndex.push_back(i);
                std::cout << "revoluteJoint: " << m_skel->getJoint(i)->getName() << std::endl;
                jointCount++;
                std::cout << "jointCount: " << jointCount << std::endl;
            }
        }
        std::cout << "revoluteJointIndex: " << revoluteJointIndex.size() << std::endl;
        return revoluteJointIndex;
    }

    //对所有旋转关节进行随机采样，返回一组关节配置
    Eigen::VectorXd getConfig(std::vector<int> revoluteJointIndex){
        int index = 0;
        for (auto i : revoluteJointIndex) { // 遍历单链关节
            auto joint = m_skel->getJoint(i);
            // 如果关节是旋转关节
            if (auto revoluteJoint = dynamic_cast<dart::dynamics::RevoluteJoint*>(joint)) {
                // 获取关节的上下限
                double lower = joint->getPositionLowerLimit(0);
                double upper = joint->getPositionUpperLimit(0);
                // 创建随机分布
                double randomAngle = generateRandom(lower, upper, 0.01);
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
    dart::dynamics::SkeletonPtr m_skel;
    int m_singlechaindofs;
    Eigen::VectorXd m_config;
    int m_revoluteJointCount;
};

//正运动学更新OcTree，每访问1000个点保存一次
void FK(octomap::OcTree& tree, dart::dynamics::SkeletonPtr skel, double newvisitpointratio, double processRatio, std::vector<int> revoluteJointIndex, std::vector<double>& samplebound, 
        dart::simulation::WorldPtr& world, Eigen::VectorXd& jointTorqueLimits, int& NewvisitPointCount, RandomJointsConfigs& randomJointsConfigs, int samplePointsSize, int timeWithoutUpdate){
    int totalvisitcount = 0; //总的遍历的点个数，无论是否有效
    int newVisitCountInBatch = 0; // 用于跟踪每1000次迭代中新访问的点数

    //计时，start_time为开始时间，interval_time为时间间隔，target_time为目标时间
    auto start_time = std::chrono::steady_clock::now();
    auto interval_time = std::chrono::minutes(timeWithoutUpdate);
    auto target_time = start_time + interval_time;

    // 开始循环
    while(true){
        // 计算当前时间与目标时间的差值
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(target_time - current_time);

        // 随机采样关节角
        Eigen::VectorXd config = randomJointsConfigs.getConfig(revoluteJointIndex);
        skel->setPositions(config);

        // 获取右手末端执行器的位置。考虑到机器人为对称的，因此右手可达的点即为左手可达的点
        auto rightEndEffector = skel->getBodyNode("rightHandEE");
        Eigen::VectorXd positions(3);
        positions = rightEndEffector->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World()).translation(); // 只获取了右手末端执行器的位置，左手末端执行器的位置没有获取
        octomap::point3d point(positions[0], positions[1], positions[2]);

        // 判断正运动学得到的点是否在空间范围内
        if (point.x() >= samplebound[0] && point.x() <= samplebound[1] && point.y() >= samplebound[2] && point.y() <= samplebound[3] && point.z() >= samplebound[4] && point.z() <= samplebound[5]){ // 判断点是否在有效范围内
            octomap::OcTreeNode* node = tree.search(point);
            Detector Detector(skel, world, jointTorqueLimits);

            // 判断之前是否已经添加过该点
            if (node == nullptr) {
                // 判断力矩是否超限，如果没有则添加该点
                bool jointTorqueFlag = Detector.checkJointTorque();
                if (!jointTorqueFlag){
                    // 对称点，对称点在y轴上对称,右手能到的点,左手也能到
                    octomap::point3d mirrorpoint = point; 
                    mirrorpoint.y() = -point.y();
                    tree.updateNode(point, true);
                    tree.updateNode(mirrorpoint, true);
                    NewvisitPointCount += 2;
                    newVisitCountInBatch += 2;
                }
            }
        }
        totalvisitcount++;

        //每循环1000个点，进行以下处理
        if (totalvisitcount%1000 == 0) {
            std::cout << "batchRatio:" << static_cast<double>(newVisitCountInBatch) << "%" << std::endl;
            std::cout << "NewvisitPointCount: " << NewvisitPointCount << " " << static_cast<double>(NewvisitPointCount)/static_cast<double>(samplePointsSize)*100 << "%" << std::endl;
            
            //经过至少一个interval_time时间
            if (duration.count() <= 0){ 
                // 如果新访问的点数不再增加，则停止循环
                if (newVisitCountInBatch == 0){
                    std::cout << "No more new points" << std::endl;
                    break;
                }
                // 如果还有新的有效点增加，则再延长一个interval_time时间，到时再判断
                target_time += interval_time;
                // 重置新访问的点数
                newVisitCountInBatch = 0;
            }

            // 保存OcTree以及NewvisitPointCount
            CtrlC::saveState();
            double batchRatio = static_cast<double>(newVisitCountInBatch) / 1000.0;
            
            //每1000个点中新访问的点的比例阈值，如果低于这个数就结束FK。最好设置比较低，接近0。设置一个较低的阈值，可以减少FK的遍历时间。因为遍历的点越多，找到剩下的有效点所需的时间就越长。设置0意味着必须找到所有的有效点才结束
            if (batchRatio < newvisitpointratio) {
                std::cout << "Batch new visit point ratio is less than newvisitpointratio: " << batchRatio << std::endl;
                break;
            }

            //设置FK遍历的有效点占离散空间的比例阈值，如果低于这个数就结束整个程序
            if (static_cast<double>(NewvisitPointCount)/static_cast<double>(samplePointsSize) >= processRatio){
                std::cout << "static_cast<double>(NewvisitPointCount)/static_cast<double>(samplePointsSize): " << static_cast<double>(NewvisitPointCount)/static_cast<double>(samplePointsSize) << std::endl;
                break;
                }
            }
    }
}

//中断处理函数，接受到ctrl+c时，保存OcTree以及NewvisitPointCount并退出程序
void signalHandler(int signal) { 
    if (signal == SIGINT) {
        //CtrlC::interrupted = true;
        std::cout << "Caught signal, exiting program..." << std::endl;
        CtrlC::saveState();
        exit(0);
    }
}

//创造点云框架
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
    //接受ctrl+c中断，保存Octree以及NewvisitPointCount并退出程序
    signal(SIGINT, signalHandler); 

    std::string treefilePath = "/home/haitaoxu/workspaceanalysis/M1_full_load_10.bt";
    std::string filename = "/home/haitaoxu/workspaceanalysis/NewvisitPointCount_load.txt";
    std::string urdfFile = "/home/haitaoxu/workspaceanalysis/models/M1/M1_full_load.urdf";

    //选择是对工作空间进行分析还是可视化
    enum ProgramProcessMode{
    ProgramProcessMode_Process = 0, //分析
    ProgramProcessMode_Visualize = 1, //可视化
    };
    ProgramProcessMode ProgramProcessMode = ProgramProcessMode_Process;

    double resolution = 0.05; //分辨率

    double newvisitpointratio = 0; //每1000个点中新访问的点的比例阈值，如果低于这个数就结束FK。最好设置比较低，接近0。设置一个较低的阈值，可以减少FK的遍历时间。因为遍历的点越多，找到剩下的有效点所需的时间就越长。设置0意味着必须找到所有的有效点才结束
    double processRatio = 0.99; //设置FK遍历的有效点占离散空间的比例阈值，如果低于这个数就结束整个程序
    int timeWithoutUpdate = 10; //分钟数，如果timeWithoutUpdate分钟内没有更新新的有效点则结束程序
    
    CtrlC::restoreState(); // 恢复上次中断时的状态
    octomap::OcTree& tree = CtrlC::tree;
    //开始对整个程序进行计时
    auto start_time = std::chrono::steady_clock::now();
    
    dart::simulation::WorldPtr world = dart::simulation::World::create();
    WorkSpaceAnalysis::WorkspaceAnalyzer analyzer(urdfFile);
    auto skel = analyzer.m_robot->GetSkeleton();
    world->addSkeleton(skel);

    //获取关节力矩限制
    Eigen::VectorXd jointTorqueLimits = analyzer.m_robot->m_robotConfig.jointTorqueLimits; 
    
    //定义要空间范围
    double x_min = 0.21, x_max = 2.21;
    double y_min = -0.3, y_max = 0.3;
    double z_min = 0.4, z_max = 0.5;
    
    //获取右手末端执行器的位置，从而设置y轴的值
    auto rh = skel->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    Eigen::Vector3d eePos = rh.translation();
    //可视化时用下面两行作为y轴的值，只显示该平面的有效点
    // y_min = eePos.y();
    // y_max = eePos.y();
    std::vector<double> samplebound = {x_min, x_max, y_min, y_max, z_min, z_max};

    //对空间进行离散化
    std::vector<Eigen::Vector3d> samplePoints;
    auto SpaceSamplerPtr = std::make_shared<SpaceSampler>();
    SpaceSamplerPtr->Discretize3DSpace(Eigen::Map<Eigen::Vector6d>(samplebound.data()), resolution, samplePoints); //这样才能得到正确的离散的所有点。如果不对空间离散化，直接在遍历时对Octree中的点进行 关节角随机采样后经过正运动学后得到的末端执行器的点 搜索并判断是否存在，是否有效等操作，NewvisitPointCount的大小会远远大于pointcloud的大小
    int samplePointsSize = samplePoints.size();
    std::cout << "samplePointsSize: " << samplePointsSize << std::endl;

    Eigen::VectorXd config = Eigen::VectorXd::Zero(skel->getNumDofs()); //注意： getNumJoints() 返回的是所有关节的数量，而 getNumDofs() 返回的是自由度的数量
    
    int singlechaindofs = 11; // 单链的自由度数量
    int revoluteJointCount = skel->getNumDofs(); //18
    RandomJointsConfigs randomJointsConfigs(skel, singlechaindofs, config, revoluteJointCount);
    std::vector<int> revoluteJointIndex = randomJointsConfigs.getrevoluteJointIndex();

    int& NewvisitPointCount = CtrlC::NewvisitPointCount;

    // 如果程序模式为处理，则进行正运动学更新OcTree
    if (ProgramProcessMode == ProgramProcessMode_Process){
        std::cout << "ProgramProcessMode_Process" << std::endl;

        if (std::filesystem::exists(treefilePath)){ // 如果树文件存在，则读取树文件
            tree.readBinary(treefilePath);
        }
        else{
            std::cout << "treefilePath not exists" << std::endl;
        }
        std::cout << "FK" << std::endl; // 计算正运动学
        FK(tree, skel, newvisitpointratio, processRatio, revoluteJointIndex, samplebound, world, jointTorqueLimits, NewvisitPointCount, randomJointsConfigs, samplePointsSize, timeWithoutUpdate);
    }

    // 如果程序模式为可视化，则进行可视化
    else if (ProgramProcessMode == ProgramProcessMode_Visualize){
        std::cout << "ProgramProcessMode_Visualize" << std::endl;

        if (std::filesystem::exists(treefilePath)){ // 如果树文件存在，则读取树文件
            tree.readBinary(treefilePath);
        }
        else{
            std::cout << "treefilePath not exists" << std::endl;
        }
    }

    // 搭建点云
    auto pointCloudFrame = createPointCloudFrame(); //createPointCloudFrame 函数创建一个 SimpleFrame，用于表示点云在仿真环境中的位置和方向。
    auto pointCloudShape = std::dynamic_pointer_cast<dart::dynamics::PointCloudShape>(pointCloudFrame->getShape());
    octomap::Pointcloud pointCloud;
    pointCloud.clear();
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> colors; //这段代码定义了一个名为 colors 的 std::vector 容器，其元素类型为 Eigen::Vector4d，同时使用了 Eigen::aligned_allocator 来管理内存对齐。以下是详细解析。
    for (auto positions:samplePoints) {
        octomap::point3d point(positions[0], positions[1], positions[2]);
        octomap::OcTreeNode* node = tree.search(point);
        if (node != nullptr) {
            if (node->getOccupancy() >= 0.5) {
            pointCloud.push_back(point.x(), point.y(), point.z());
            colors.push_back({0, 0, 1, 0.5});
            }
        }
    }

    std::cout << "pcl pointCloud.size(): " << pointCloud.size() << std::endl;
    std::cout << "Space sampled point size:"<< samplePointsSize << std::endl;
    std::cout << "Valid Points in Space Ratio:"<< static_cast<double>(pointCloud.size())/static_cast<double>(samplePointsSize)*100 << "%" << std::endl;
    pointCloudShape->setVisualSize(resolution);

    // 添加工作区间的边界线
    // SpaceSampler sampler;
    // std::vector<Eigen::Vector3d> boundaryPoints = SpaceSamplerPtr->get2dBoundaryPoints(resolution, samplebound);

    // for (auto point : boundaryPoints) {
    //     pointCloud.push_back(point.x(), point.y(), point.z());
    //     colors.push_back({1, 0, 0, 0.5}); 
    // }
    assert(pointCloud.size() == colors.size());
    pointCloudShape->setColors(colors);
    pointCloudShape->setPoints(pointCloud);
    world->addSimpleFrame(pointCloudFrame);

    // 最终可视化设置 -------------------------------------------------------------------------------------------------------
    
    // 可视化时skel变回零位
    skel->setPositions(Eigen::VectorXd(revoluteJointCount).setZero());
    skel->getJoint(0)->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());

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

    //3D可视化
    viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    viewer->getCameraManipulator()->setHomePosition(::osg::Vec3(xAxisOffset, eyeY, zAxisOffset),::osg::Vec3(xAxisOffset, 0.0, zAxisOffset), up_xzView);
    viewer->setCameraManipulator(viewer->getCameraManipulator());

    //2D可视化
    // viewer->getCameraManipulator()->setHomePosition(::osg::Vec3(xAxisOffset, eyeY, zAxisOffset),::osg::Vec3(xAxisOffset, 0.0, zAxisOffset), up_xzView);
    // viewer->setCameraManipulator(viewer->getCameraManipulator());

    std::cout << "save tree.writeBinary success" << std::endl;
    CtrlC::saveState();

    // 计算整个程序运行的时间，输出结果为毫秒
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);  
    std::cout << "Elapsed time: " << duration.count() << " milliseconds\n";

    viewer->run();
    

    return 0;
}