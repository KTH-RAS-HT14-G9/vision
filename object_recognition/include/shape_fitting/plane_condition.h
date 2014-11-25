#ifndef PLANE_CONDITION_H
#define PLANE_CONDITION_H

#include <pcl/ModelCoefficients.h>
#include <Eigen/Core>

class PlaneCondition {
public:
    PlaneCondition() {}

    virtual bool condition(const std::vector<pcl::ModelCoefficients>& planes, const std::vector<Eigen::Vector4f>& centroids){
        return true;
    }
};

#endif // PLANE_CONDITION_H
