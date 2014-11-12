#include "shape_fitting/shape_classifier_base.h"

ShapeClassifierBase::ShapeClassifierBase(const std::string &name)
{
    _name = name;
}

void ShapeClassifierBase::visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients& coefficients)
{

}

double ShapeClassifierBase::classify(const common::SharedPointCloudRGB &cloud, pcl::ModelCoefficients::Ptr &coefficients)
{
    return 0;
}

std::string ShapeClassifierBase::name()
{
    return _name;
}
