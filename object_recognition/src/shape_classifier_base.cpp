#include "shape_fitting/shape_classifier_base.h"

ShapeClassifierBase::ShapeClassifierBase(const std::string &name)
{
    _name = name;
}

#ifdef ENABLE_VISUALIZATION_RECOGNITION
void ShapeClassifierBase::visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients& coefficients)
{

}
#endif

common::Classification ShapeClassifierBase::classify(const common::SharedPointCloudRGB &cloud, pcl::ModelCoefficients::Ptr &coefficients)
{
    return common::Classification();
}

std::string ShapeClassifierBase::name()
{
    return _name;
}
