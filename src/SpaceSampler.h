#include <Eigen/Core>
#include <Eigen/Dense>
#include <random>
#include <dart/dart.hpp>

class SpaceSampler {
public:
    /**
     * @brief 离散化角度空间，生成以centerAngle为中心，正负交替的角度范围为[-maxAngle,
     * maxAngle]的角度序列，如{0，0.1，-0.1，0.2，-0.2，...}
     *
     * @param centerAngle 离散化的中心角度
     * @param maxAngle 角度序列的最大绝对值
     * @param resolution 离散化的分辨率
     * @param sampleAngles 输出的角度序列
     */
    void DiscretizeSingleAngleSpace(double centerAngle, double maxAngle, double resolution,
                                    std::vector<double>& sampleAngles);

    void Discretize1DSpace(Eigen::Vector2d boundaries, double resolution, std::vector<double>& samplePoints);

    void Discretize3DSpace(Eigen::Vector6d boundaries, double resolution, std::vector<Eigen::Vector3d>& samplePoints);

private:
};
