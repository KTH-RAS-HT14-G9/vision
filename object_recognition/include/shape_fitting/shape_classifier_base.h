#ifndef SHAPE_CLASSIFIER_BASE_H
#define SHAPE_CLASSIFIER_BASE_H

#include <common/classification.h>
#include <common/debug.h>
#include <pcl/ModelCoefficients.h>

#ifdef ENABLE_VISUALIZATION_RECOGNITION
#include <pcl/visualization/pcl_visualizer.h>
#endif

class ShapeClassifierBase
{
public:
    ShapeClassifierBase(const std::string& name);
    virtual common::Classification classify(const common::SharedPointCloudRGB& cloud, pcl::ModelCoefficients::Ptr& coefficients);

#ifdef ENABLE_VISUALIZATION_RECOGNITION
    virtual void visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients& coefficients);
#endif

    std::string name();

protected:
    std::string _name;
};

#endif // SHAPE_CLASSIFIER_BASE_H
