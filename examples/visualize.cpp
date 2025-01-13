#include <iostream>
#include <fstream>
#include <octomap/octomap.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/utils/utils.hpp>
#include <dart/simulation/World.hpp>
#include <dart/dynamics/SmartPointer.hpp>
#include <Eigen/Dense>
#include <vector>
#include <dart/gui/osg/osg.hpp>
#include <osgGA/TrackballManipulator>
#include <filesystem> 
#include "SpaceSampler.h"

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
    std::string urdfFile = "/home/haitaoxu/workspaceanalysis/models/M1/M1_full.urdf";

    dart::simulation::WorldPtr world = dart::simulation::World::create();
    dart::utils::DartLoader urdf;
    dart::dynamics::SkeletonPtr m1 = urdf.parseSkeleton(urdfFile);  // 加载 URDF 文件
    world->addSkeleton(m1);

    double resolution = 0.01;

    double x_min = -0.2, x_max = 2.0;
    double y_min, y_max;
    double z_min = 0.0, z_max = 2.0;

    auto rh = m1->getBodyNode("rightHandEE")->getTransform(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
    Eigen::Vector3d eePos = rh.translation();
    // y_min = eePos.y()-resolution;
    // y_max = eePos.y()+resolution;
    y_min = eePos.y();
    y_max = eePos.y();

    // y_min = eePos.y()-resolution;
    // y_max = eePos.y()+resolution;
    // y_min = eePos.y();
    // y_max = eePos.y();
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
    std::cout << "samplebound"  << std::endl;

    std::vector<Eigen::Vector3d> samplePoints;
    auto SpaceSamplerPtr = std::make_shared<SpaceSampler>();
    SpaceSamplerPtr->Discretize3DSpace(Eigen::Map<Eigen::Vector6d>(samplebound.data()), resolution, samplePoints); //这样才能得到正确的离散的所有点
    int samplePointsSize = samplePoints.size();
    std::cout << "samplePointsSize: " << samplePointsSize << std::endl;

    octomap::OcTree tree(resolution);
    tree.read(treefilePath);

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
    m1->setPositions(Eigen::VectorXd(m1->getNumDofs()).setZero());
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

    viewer->setCameraManipulator(new osgGA::TrackballManipulator()); //3d 
    viewer->getCameraManipulator()->setHomePosition(::osg::Vec3(xAxisOffset, eyeY, zAxisOffset),::osg::Vec3(xAxisOffset, 0.0, zAxisOffset), up_xzView);
    viewer->setCameraManipulator(viewer->getCameraManipulator());

    // viewer->getCameraManipulator()->setHomePosition(::osg::Vec3(xAxisOffset, eyeY, zAxisOffset),::osg::Vec3(xAxisOffset, 0.0, zAxisOffset), up_xzView); //2d
    // viewer->setCameraManipulator(viewer->getCameraManipulator());


    viewer->run();
    

    return 0;
}

