#ifndef PLANE_FITTING_H
#define PLANE_FITTING_H

#include <shape_fitting/shape_classifier_base.h>
#include <common/parameter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include "plane_condition.h"

class PlaneFitting : public ShapeClassifierBase
{
public:
    PlaneFitting(const std::string& name,
                 int min_planes,
                 const std::string& parameter_prefix,
                 PlaneCondition* condition);

    virtual common::Classification classify(const common::SharedPointCloudRGB &cloud, pcl::ModelCoefficients::Ptr &coefficients);

#ifdef ENABLE_VISUALIZATION_RECOGNITION
    virtual void visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients &coefficients);
#endif

protected:


    double rectangular_measure(std::vector<pcl::ModelCoefficients> planes);
    int rebuild_indices_for_inlier_flag(pcl::PointIndices::Ptr& indices, std::vector<bool> flags, bool condition);
    void set_parameters();
    void build_multiple_plane_model(std::vector<pcl::ModelCoefficients>& planes, pcl::ModelCoefficients::Ptr& coefficients);

    PlaneCondition* _condition;

    int _min_planes;
    //pcl::SACSegmentation<pcl::PointXYZRGB> _seg;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> _seg;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> _normal_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _kd_tree;

    pcl::PointCloud<pcl::Normal>::Ptr _normals;

    pcl::PointIndices::Ptr _indices;
    pcl::PointIndices::Ptr _inliers;
    std::vector<bool> _all_inliers;
    std::vector<pcl::ModelCoefficients> _planes;
    std::vector<Eigen::Vector4f> _centroids;

    Parameter<double> _distance_threshold;
    Parameter<double> _halt_condition;
    Parameter<int> _kd_k;
    Parameter<double> _normal_weight;
};

#endif // PLANE_FITTING_H
