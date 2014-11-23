#ifndef PLANE_FITTING_H
#define PLANE_FITTING_H

#include <shape_fitting/shape_classifier_base.h>
#include <common/parameter.h>
#include <pcl/segmentation/sac_segmentation.h>

class PlaneFitting : public ShapeClassifierBase
{
public:
    PlaneFitting(const std::string& name,
                 int min_planes,
                 const std::string& parameter_prefix,
                 bool (*condition)(const std::vector<pcl::ModelCoefficients>&, const std::vector<Eigen::Vector4f>&));

    virtual common::NameAndProbability classify(const common::SharedPointCloudRGB &cloud, pcl::ModelCoefficients::Ptr &coefficients);

    virtual void visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients &coefficients);

protected:


    double rectangular_measure(std::vector<pcl::ModelCoefficients> planes);
    int rebuild_indices_for_inlier_flag(pcl::PointIndices::Ptr& indices, std::vector<bool> flags, bool condition);
    void set_parameters();
    void build_multiple_plane_model(std::vector<pcl::ModelCoefficients>& planes, pcl::ModelCoefficients::Ptr& coefficients);

    bool (*_condition)(const std::vector<pcl::ModelCoefficients>&, const std::vector<Eigen::Vector4f>&);

    int _min_planes;
    pcl::SACSegmentation<pcl::PointXYZRGB> _seg;
    pcl::PointIndices::Ptr _indices;
    pcl::PointIndices::Ptr _inliers;
    std::vector<bool> _all_inliers;
    std::vector<pcl::ModelCoefficients> _planes;
    std::vector<Eigen::Vector4f> _centroids;

    Parameter<double> _distance_threshold;
    Parameter<double> _halt_condition;
};

#endif // PLANE_FITTING_H
