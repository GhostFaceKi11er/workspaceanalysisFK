#include "SpaceSampler.h"

void SpaceSampler::DiscretizeSingleAngleSpace(double centerAngle, double maxAngle, double resolution, std::vector<double>& sampleAngles) { //DiscretizeSingleAngleSpace 是一个用于离散化单一角度空间的函数。它以某个中心角度为基准，通过指定的分辨率向两侧生成一系列离散的角度值。
    sampleAngles.push_back(centerAngle);
    double angle = centerAngle;
    for (int i = 1; angle + i*resolution <= maxAngle; i++) {
        sampleAngles.push_back(angle + i*resolution);
        sampleAngles.push_back(angle - i*resolution);
    }
}

void SpaceSampler::Discretize1DSpace(Eigen::Vector2d boundaries, double resolution, std::vector<double>& samplePoints) { //这段代码定义了一个名为 Discretize1DSpace 的成员函数，用于对一维空间进行离散化处理。
    for (double i = boundaries(0); i <= boundaries(1) + 1e-9; i += resolution) {                                         //它根据指定的范围（boundaries）和分辨率（resolution），生成一系列离散点，并存储在一个向量（samplePoints）中。离散的是角度
        samplePoints.push_back(i);
    }
}

void SpaceSampler::Discretize3DSpace(Eigen::Vector6d boundaries, double resolution, std::vector<Eigen::Vector3d>& samplePoints) { //离散化距离
    for (double i = boundaries(0); i <= boundaries(1) + 1e-9; i += resolution) {
        for (double j = boundaries(2); j <= boundaries(3) + 1e-9; j += resolution) {
            for (double k = boundaries(4); k <= boundaries(5) + 1e-9; k += resolution) {
                samplePoints.push_back(Eigen::Vector3d(i, j, k));
            }
        }
    }
}

