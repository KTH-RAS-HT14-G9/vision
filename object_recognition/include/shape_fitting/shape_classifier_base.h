#ifndef SHAPE_CLASSIFIER_BASE_H
#define SHAPE_CLASSIFIER_BASE_H

#include <common/types.h>
#include <common/debug.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>

class ShapeClassifierBase
{
public:
    ShapeClassifierBase(const std::string& name);
    virtual double classify(const common::SharedPointCloudRGB& cloud, pcl::ModelCoefficients::Ptr& coefficients);

    virtual void visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients& coefficients);

    std::string name();

protected:
    std::string _name;
};

#endif // SHAPE_CLASSIFIER_BASE_H
