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
#include <Eigen/Geometry>
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

class Detector{
public:
    Detector(dart::dynamics::SkeletonPtr& skel, dart::simulation::WorldPtr& world, Eigen::VectorXd& jointTorqueLimits): m_skel(skel), m_world(world), m_jointTorqueLimits(jointTorqueLimits) {}

    bool checkCollision(){
        auto Detector = m_world->getConstraintSolver()->getCollisionDetector(); // 检测自碰撞    
        auto collisionGroup = Detector->createCollisionGroup(m_skel.get()); //创建一个碰撞组（CollisionGroup），包含机器人所有的碰撞几何体
        dart::collision::CollisionOption option;  //碰撞检测选项（如是否需要详细结果）
        dart::collision::CollisionResult result;  //碰撞检测结果（如发生碰撞的物体对、碰撞点等）。

        bool inCollision = collisionGroup->collide(option, &result);
        if (inCollision) return true; 
        else return false;
        }

    bool checkJointTorque() { // 逆动力学计算每个关节的力矩，如果力矩超限则输出true，不超限则输出false
        m_skel->computeInverseDynamics();
        Eigen::VectorXd jointForces = m_skel->getForces();
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
    dart::dynamics::SkeletonPtr m_skel;
    dart::simulation::WorldPtr m_world; 
    Eigen::VectorXd m_jointTorqueLimits;
};

class RandomJointsConfigs{
public:
    RandomJointsConfigs(const dart::dynamics::SkeletonPtr& skel, const int& singlechaindofs, Eigen::VectorXd& config, const int& revoluteJointCount): 
    m_skel(skel), m_singlechaindofs(singlechaindofs), m_config(config), m_revoluteJointCount(revoluteJointCount) {}
    
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
        //std::cout << "skel->getNumDofs(): " << skel->getNumDofs() << std::endl;
        for (size_t i = 0; i < m_skel->getNumJoints(); ++i) { // 遍历单链关节
            auto joint = m_skel->getJoint(i);
            if (jointCount >= m_singlechaindofs) break;
            if (auto revoluteJoint = dynamic_cast<dart::dynamics::RevoluteJoint*>(joint)){
                //revoluteJointCount++;
                revoluteJointIndex.push_back(i);
                std::cout << "revoluteJoint: " << m_skel->getJoint(i)->getName() << std::endl;
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
            auto joint = m_skel->getJoint(i);
            if (auto revoluteJoint = dynamic_cast<dart::dynamics::RevoluteJoint*>(joint)) { // 如果关节是旋转关节
                //std::cout << "Sampling for joint: " << joint->getName() << std::endl;
                // 获取关节的上下限
                double lower = joint->getPositionLowerLimit(0);
                double upper = joint->getPositionUpperLimit(0);
                // 创建随机分布
                //std::uniform_real_distribution<> dis(lower, upper);
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

bool FK(octomap::OcTree& tree, dart::dynamics::SkeletonPtr skel, double newvisitpointratio, double processRatio, std::vector<int> revoluteJointIndex, std::vector<double>& samplebound, 
        dart::simulation::WorldPtr& world, Eigen::VectorXd& jointTorqueLimits, int& NewvisitPointCount, RandomJointsConfigs& randomJointsConfigs, int samplePointsSize){
    int totalvisitcount = 0; //总的遍历的点个数
    int NewvisitPointCountInBatch = 0; // 用于跟踪每1000次迭代中新访问的点数
    double constant = 2;
    //int treeSize = tree.size();

    auto start_time = std::chrono::steady_clock::now();
    auto interval_time = std::chrono::minutes(20);
    auto target_time = start_time + interval_time;
    while(true){
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(target_time - current_time);
        //std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;

        Eigen::VectorXd config = randomJointsConfigs.getConfig(revoluteJointIndex);
        skel->setPositions(config);
        auto rightEndEffector = skel->getBodyNode("rightHandEE");
        Eigen::VectorXd positions(3);
        positions = rightEndEffector->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World()).translation(); // 只获取了右手末端执行器的位置，左手末端执行器的位置没有获取
        octomap::point3d point(positions[0], positions[1], positions[2]);

        if (point.x() >= samplebound[0] && point.x() <= samplebound[1] && point.y() >= samplebound[2] && point.y() <= samplebound[3] && point.z() >= samplebound[4] && point.z() <= samplebound[5]){ // 判断点是否在有效范围内
            octomap::OcTreeNode* node = tree.search(point);
            Detector Detector(skel, world, jointTorqueLimits);

            bool jointTorqueFlag = Detector.checkJointTorque();
            if (node == nullptr  && !jointTorqueFlag) {
                tree.updateNode(point, true);
                NewvisitPointCount++;
                NewvisitPointCountInBatch++;
            }
            octomap::point3d mirrorpoint = point; // 对称点, 对称点在y轴上对称,右手能到的点,左手也能到
            mirrorpoint.y() = -point.y();
            octomap::OcTreeNode* mirrornode = tree.search(mirrorpoint);
            if (mirrornode == nullptr && !jointTorqueFlag) {
                tree.updateNode(mirrorpoint, true);
                NewvisitPointCount++;
            }
        }
        totalvisitcount++;
        if (totalvisitcount%1000 == 0) {
            std::cout << "batchRatio:" << static_cast<double>(NewvisitPointCountInBatch) << "%" << std::endl;
            std::cout << "NewvisitPointCount: " << NewvisitPointCount << " " << static_cast<double>(NewvisitPointCount)/static_cast<double>(samplePointsSize)*100 << "%" << std::endl;
            
            if (duration.count() == 0){ //经过一个interval_time时间
                if (NewvisitPointCountInBatch == 0){ // 如果新访问的点数不再增加，则停止循环
                    std::cout << "No more new points" << std::endl;
                    return false;
                }
                target_time += interval_time; //如果还有新的有效点增加，则再延长一个interval_time时间，到时再判断
                //NewvisitPointCountInBatch = 0; //重置新访问的点数
            }

            CtrlC::saveState();
            double batchRatio = static_cast<double>(NewvisitPointCountInBatch) / 1000;
            if (batchRatio < newvisitpointratio) {
                std::cout << "Batch new visit point ratio is less than newvisitpointratio: " << batchRatio << std::endl;
                return true;
            }
            NewvisitPointCountInBatch = 0; // 重置批次计数器
            if (static_cast<double>(NewvisitPointCount)/static_cast<double>(samplePointsSize) >= processRatio){ //如果有效点占空间的比例超过processRatio则停止FK
                std::cout << "static_cast<double>(NewvisitPointCount)/static_cast<double>(samplePointsSize): " << static_cast<double>(NewvisitPointCount)/static_cast<double>(samplePointsSize) << std::endl;
                return false;
                }
            }
    }
    return false;
}

void signalHandler(int signal) { //Ctrl+c 中断处理函数
    if (signal == SIGINT) {
        //CtrlC::interrupted = true;
        std::cout << "Caught signal, exiting program..." << std::endl;
        CtrlC::saveState();
        exit(0);
    }
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


class TWOTCPIK{
public:
    TWOTCPIK(octomap::OcTree& tree, dart::dynamics::SkeletonPtr skel, dart::simulation::WorldPtr& world, Eigen::VectorXd& jointTorqueLimits, 
        int& NewvisitPointCount, std::vector<Eigen::Vector3d>& samplePoints, int samplePointsSize): m_tree(tree), m_skel(skel), m_world(world), m_jointTorqueLimits(jointTorqueLimits), 
        m_NewvisitPointCount(NewvisitPointCount), m_samplePoints(samplePoints), m_samplePointsSize(samplePointsSize) {}

    Eigen::MatrixXd getRHjacobian(){
        dart::dynamics::BodyNodePtr endEffector = m_skel->getBodyNode("rightHandEE");
        Eigen::MatrixXd jacobian = m_skel->getJacobian(endEffector); //获得的是6*18的矩阵，包含了整个机器人的活动关节
        jacobian = jacobian.topRows(3);
        return jacobian;
    }

    Eigen::MatrixXd getLHjacobian(){
        dart::dynamics::BodyNodePtr endEffector = m_skel->getBodyNode("leftHandEE");
        Eigen::MatrixXd jacobian = m_skel->getJacobian(endEffector); //获得的是6*18的矩阵，包含了整个机器人的活动关节
        jacobian = jacobian.topRows(3);

        return jacobian;
    }
    
    Eigen::VectorXd getRHtheta(){
        Eigen::VectorXd theta = m_skel->getPositions(); //大小为18
        return theta;
    }

    Eigen::VectorXd getLHtheta(){
        Eigen::VectorXd theta = m_skel->getPositions(); //大小为18
        return theta;
    }

    void IK(){
        std::cout << "IK" << std::endl;
        std::cout << "NewvisitPointCount: " << m_NewvisitPointCount << std::endl;
        Eigen::Isometry3d rightHandTf;
        Eigen::Isometry3d leftHandTf;
        double epsilon = 1e-4;
        double alpha_steplenth = 0.6;
        int iteration = 0;
        int newpointcount = 0;

        auto start_time_ik = std::chrono::steady_clock::now();
        auto interval_time_ik = std::chrono::minutes(5);
        auto target_time_ik = start_time_ik + interval_time_ik;
        while (m_NewvisitPointCount < m_samplePointsSize){
            auto current_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(target_time_ik - current_time);
            if (duration.count() <= 0){ //至少每一个interval_time_ik检查一次
                if (newpointcount == 0){
                    break;
                }
                target_time_ik = current_time + interval_time_ik;
                newpointcount = 0;
            }
            for (int i = 0; i < m_samplePoints.size(); i++){
                Eigen::Vector3d samplepoint = m_samplePoints[i];
                //std::cout << "i: " << i << std::endl;
                //设置末端执行器的目标位置
                rightHandTf.translation() = samplepoint;
                leftHandTf.translation() = samplepoint;
                leftHandTf.translation()[1] = -rightHandTf.translation()[1]; //左手和右手的y的值只是正负号的区别
                
                Eigen::MatrixXd rhJacobian = getRHjacobian(); //3*18
                Eigen::MatrixXd lhJacobian = getLHjacobian(); //3*18

                Eigen::VectorXd RHtheta = getRHtheta();
                Eigen::VectorXd LHtheta = getLHtheta();
                // Eigen::VectorXd wholetheta(LHtheta.size()+RHtheta.size());
                // wholetheta << LHtheta, RHtheta;
                Eigen::VectorXd wholetheta(LHtheta.size());
                //std::cout << "wholetheta.size(): " << wholetheta.size() << std::endl;

                while (iteration < 1000){
                    Eigen::Isometry3d LHeePose = m_skel->getBodyNode("leftHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
                    Eigen::Isometry3d RHeePose = m_skel->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
                    //计算位置误差
                    Eigen::Vector3d RHtranslation_error = rightHandTf.translation() - RHeePose.translation();
                    Eigen::Vector3d LHtranslation_error = leftHandTf.translation() - LHeePose.translation();
                    //std::cout << "RHtranslation_error: " << RHtranslation_error << std::endl;
                    Eigen::VectorXd translation_error(6);
                    translation_error << LHtranslation_error, RHtranslation_error;

                    Eigen::MatrixXd wholeJacobian(lhJacobian.rows()+rhJacobian.rows(), lhJacobian.cols()); //6*18
                    wholeJacobian << lhJacobian, rhJacobian;
                    //std::cout << "wholeJacobian.rows(): " << wholeJacobian.rows() << "  and .cols():"<< wholeJacobian.cols() << "translation_error.size(): " << translation_error.size() << std::endl;
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd(wholeJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
                    Eigen::MatrixXd J_pseudo_inverse = svd.solve(Eigen::MatrixXd::Identity(wholeJacobian.rows(), wholeJacobian.rows()));

                    //std::cout << "J_pseudo_inverse.rows() and .cols(): " << J_pseudo_inverse.rows() << "  :"<< J_pseudo_inverse.cols() << "translation_error.size(): " << translation_error.size() << std::endl;
                    // 计算关节增量
                    Eigen::VectorXd delta_theta = J_pseudo_inverse * translation_error;
                    wholetheta += alpha_steplenth * delta_theta;
                    // Eigen::VectorXd theta(skel->getPositions().size());
                    // theta = wholetheta.segment(0,11) + wholetheta.segment(14, 7);
                    //std::cout << "wholetheta.size(): " << wholetheta.size() << std::endl;
                    m_skel->setPositions(wholetheta);
                    //std::cout << "delta_theta.norm(): " << delta_theta.norm() << std::endl;
                    if (delta_theta.norm() <= epsilon){
                        octomap::point3d point(samplepoint[0], samplepoint[1], samplepoint[2]);
                        if (m_tree.search(point) == nullptr){
                            Detector Detector(m_skel, m_world, m_jointTorqueLimits);
                            bool jointTorqueFlag = Detector.checkJointTorque();
                            if (!jointTorqueFlag){
                                m_tree.updateNode(point, true);
                                m_NewvisitPointCount++;
                                newpointcount++;
                                std::cout << "i: "<< i <<" NewvisitPointCount: " << m_NewvisitPointCount << " newpointcount:" << newpointcount << " duration: " << duration.count() << " point: " << point << std::endl;
                                iteration = 1000;
                            }
                        }
                    }
                    iteration++;
                }
                iteration = 0;
            }
        }
        std::cout << "IK end" << std::endl;
}
private:
    octomap::OcTree& m_tree;
    dart::dynamics::SkeletonPtr& m_skel;
    dart::simulation::WorldPtr& m_world;
    Eigen::VectorXd& m_jointTorqueLimits; 
    int& m_NewvisitPointCount;
    std::vector<Eigen::Vector3d>& m_samplePoints;
    int& m_samplePointsSize;

};


int main() {
    signal(SIGINT, signalHandler); 
    std::string treefilePath = "/home/haitaoxu/workspaceanalysis/M1_full_load_10.bt";
    std::string filename = "/home/haitaoxu/workspaceanalysis/state.txt";
    std::string urdfFile = "/home/haitaoxu/workspaceanalysis/models/M1/M1_full_load.urdf";

    //选择是对工作空间进行分析还是可视化
    enum ProgramProcessMode{
    ProgramProcessMode_Process = 0,
    ProgramProcessMode_Visualize = 1,
    };
    ProgramProcessMode ProgramProcessMode = ProgramProcessMode_Process;

    double resolution = 0.05; //

    double newvisitpointratio = 0.2; //每1000个点中新访问的点的比例阈值，如果低于这个数就结束FK，开始IK。最好设置0,使用FK
    double processRatio = 0.99; //设置FK遍历有效点占整个指定空间的比例阈值，如果低于这个数就结束整个程序

    // 恢复上次中断时的状态
    CtrlC::restoreState(); 
    octomap::OcTree& tree = CtrlC::tree;
    //开始对整个程序进行计时
    auto start_time = std::chrono::steady_clock::now();
    
    dart::simulation::WorldPtr world = dart::simulation::World::create();
    WorkSpaceAnalysis::WorkspaceAnalyzer analyzer(urdfFile);
    auto skel = analyzer.m_robot->GetSkeleton();
    world->addSkeleton(skel);

    Eigen::VectorXd jointTorqueLimits = analyzer.m_robot->m_robotConfig.jointTorqueLimits; // 获取关节力矩限制
    
    //定义要分析的空间范围
    double x_min = 0.6, x_max = 0.8;
    double y_min = -0.3, y_max = 0.3;
    double z_min = 0.5, z_max = 0.7;
    
    //可视化时获取右手末端执行器的位置，从而设置y轴的值，只显示该平面的有效点
    // auto rh = skel->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    // Eigen::Vector3d eePos = rh.translation();
    // y_min = eePos.y();
    // y_max = eePos.y();
    std::vector<double> samplebound = {x_min, x_max, y_min, y_max, z_min, z_max};

    //对空间进行离散化
    std::vector<Eigen::Vector3d> samplePoints;
    auto SpaceSamplerPtr = std::make_shared<SpaceSampler>();
    SpaceSamplerPtr->Discretize3DSpace(Eigen::Map<Eigen::Vector6d>(samplebound.data()), resolution, samplePoints); //这样才能得到正确的离散的所有点
    int samplePointsSize = samplePoints.size();
    std::cout << "samplePointsSize: " << samplePointsSize << std::endl;
    for (int i = 0; i < samplePoints.size(); i++){
        std::cout << "samplePoints[i]: " << samplePoints[i].transpose() << std::endl;
    }
    //对角度进行离散化
    std::vector<double> sampledAngles;
    SpaceSamplerPtr->DiscretizeSingleAngleSpace(0, 1.57, 0.1, sampledAngles);


    Eigen::VectorXd config = Eigen::VectorXd::Zero(skel->getNumDofs()); //注意： getNumJoints() 返回的是所有关节的数量，而 getNumDofs() 返回的是自由度的数量
    
    //整个机器人的关节有25个左右，其中revolute关节18个。因为只处理revolute关节，所以要专门获得所有revolute关节的index
    int singlechaindofs = 11; // 单链的自由度数量
    int revoluteJointCount = skel->getNumDofs(); //18
    RandomJointsConfigs randomJointsConfigs(skel, singlechaindofs, config, revoluteJointCount);
    std::vector<int> revoluteJointIndex = randomJointsConfigs.getrevoluteJointIndex();

    int& NewvisitPointCount = CtrlC::NewvisitPointCount;

    if (ProgramProcessMode == ProgramProcessMode_Process){
        std::cout << "ProgramProcessMode_Process" << std::endl;

        if (std::filesystem::exists(treefilePath))  tree.readBinary(treefilePath); // 如果树文件存在，则读取树文件
        else std::cout << "treefilePath not exists" << std::endl;

        //std::cout << "FK" << std::endl; // 计算正运动学
        //bool IKflag = FK(tree, skel, newvisitpointratio, processRatio, revoluteJointIndex, samplebound, world, jointTorqueLimits, NewvisitPointCount, randomJointsConfigs, samplePointsSize);
        bool IKflag = true;
        if (IKflag) { // 如果计算正运动学遍历不完，则继续进行逆运动学
            TWOTCPIK TWOTCPIK(tree, skel, world, jointTorqueLimits, NewvisitPointCount, samplePoints, samplePointsSize);
            TWOTCPIK.IK();
        }
    }
    else if (ProgramProcessMode == ProgramProcessMode_Visualize){
        std::cout << "ProgramProcessMode_Visualize" << std::endl;

        if (std::filesystem::exists(treefilePath)) tree.readBinary(treefilePath); // 如果树文件存在，则读取树文件
        else std::cout << "treefilePath not exists" << std::endl;
    }

    auto pointCloudFrame = createPointCloudFrame(); //createPointCloudFrame 函数创建一个 SimpleFrame，用于表示点云在仿真环境中的位置和方向。
    auto pointCloudShape = std::dynamic_pointer_cast<dart::dynamics::PointCloudShape>(pointCloudFrame->getShape());
    
    octomap::Pointcloud pointCloud;
    pointCloud.clear();
    int index = 0;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> colors; //这段代码定义了一个名为 colors 的 std::vector 容器，其元素类型为 Eigen::Vector4d，同时使用了 Eigen::aligned_allocator 来管理内存对齐。以下是详细解析。
    for (auto positions : samplePoints) {
        octomap::point3d point(positions[0], positions[1], positions[2]);
        octomap::OcTreeNode* node = tree.search(point);
        if (node != nullptr) {
            std::cout << "node->getOccupancy(): " << node->getOccupancy() << std::endl;
            if (node->getOccupancy() >= 0.5) {
                index++;
                std::cout << "point: " << point << std::endl;
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
    SpaceSampler sampler;
    std::vector<Eigen::Vector3d> boundaryPoints = SpaceSamplerPtr->get2dBoundaryPoints(resolution, samplebound);

    for (auto point : boundaryPoints) {
        pointCloud.push_back(point.x(), point.y(), point.z());
        colors.push_back({1, 0, 0, 0.5}); 
    }
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

    //3d视图
    viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    viewer->getCameraManipulator()->setHomePosition(::osg::Vec3(xAxisOffset, eyeY, zAxisOffset),::osg::Vec3(xAxisOffset, 0.0, zAxisOffset), up_xzView);
    viewer->setCameraManipulator(viewer->getCameraManipulator());

    //2d视图
    // viewer->getCameraManipulator()->setHomePosition(::osg::Vec3(xAxisOffset, eyeY, zAxisOffset),::osg::Vec3(xAxisOffset, 0.0, zAxisOffset), up_xzView);
    // viewer->setCameraManipulator(viewer->getCameraManipulator());

    std::cout << "save tree.writeBinary success" << std::endl;
    CtrlC::saveState();


    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);  // 计算时间差
    std::cout << "Elapsed time: " << duration.count() << " milliseconds\n";  // 输出结果，单位是毫秒

    viewer->run();
    

    return 0;
}

