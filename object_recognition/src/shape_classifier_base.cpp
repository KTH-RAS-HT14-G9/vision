#include "shape_fitting/shape_classifier_base.h"

ShapeClassifierBase::ShapeClassifierBase(const std::string &name)
{
    _name = name;
}

void ShapeClassifierBase::visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients& coefficients)
{

}

common::Classification ShapeClassifierBase::classify(const common::SharedPointCloudRGB &cloud, pcl::ModelCoefficients::Ptr &coefficients)
{
    return common::Classification();
}

std::string ShapeClassifierBase::name()
{
    return _name;
}
