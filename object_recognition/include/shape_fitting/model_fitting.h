#ifndef MODEL_FITTING_H
#define MODEL_FITTING_H

#include <shape_fitting/shape_classifier_base.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <common/types.h>
#include <common/parameter.h>

class ModelFitting : public ShapeClassifierBase
{
public:
    ModelFitting(const std::string& name, pcl::SacModel model, const std::string& parameter_prefix);

    virtual common::NameAndProbability classify(const common::SharedPointCloudRGB& cloud, pcl::ModelCoefficients::Ptr& coefficients);

    virtual void visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients& coefficients);

    void set_parameter();

protected:
    pcl::SacModel _model;

    pcl::PointIndices _inliers;

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> _seg;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> _normal_est;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _kd_tree;
    pcl::PointCloud<pcl::Normal>::Ptr _normals;


    Parameter<double> _min_r;
    Parameter<double> _max_r;
    Parameter<int> _kd_k;
    Parameter<double> _dist_thresh;
    Parameter<double> _normal_weight;
};

#endif // MODEL_FITTING_H
