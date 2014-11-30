#include "shape_fitting/model_fitting.h"

ModelFitting::ModelFitting(const std::string &name, pcl::SacModel model, const std::string &parameter_prefix)
    :ShapeClassifierBase(name)
    ,_model(model)
    ,_min_r(parameter_prefix+"min_radius", 0.03/2.0)
    ,_max_r(parameter_prefix+"max_radius", 0.07/2.0)
    ,_kd_k(parameter_prefix+"kd_k", 50)
    ,_dist_thresh(parameter_prefix+"dist_thresh", 0.002)
    ,_normal_weight(parameter_prefix+"normal_dist_weight", 0.1)
{
    // Optional
    _seg.setOptimizeCoefficients (true);
    // Mandatory
    _seg.setModelType (model);
    _seg.setMethodType(pcl::SAC_RANSAC);

    _normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    _kd_tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);

    _normal_est.setSearchMethod(_kd_tree);
}

void ModelFitting::set_parameter()
{
    _seg.setDistanceThreshold(_dist_thresh());
    _seg.setNormalDistanceWeight(_normal_weight());
    _seg.setRadiusLimits(_min_r(),_max_r());

    _normal_est.setKSearch(_kd_k());
}

common::Classification ModelFitting::classify(const common::SharedPointCloudRGB &cloud, pcl::ModelCoefficients::Ptr &coefficients)
{
    set_parameter();

    _normals->clear();
    _inliers.indices.clear();
    coefficients->values.clear();

    _normal_est.setInputCloud(cloud);
    _normal_est.compute(*_normals);

    _seg.setInputCloud(cloud);
    _seg.setInputNormals(_normals);
    _seg.segment(_inliers,*coefficients);

    double nPoints = (double)cloud->size();
    double nInliers = (double)_inliers.indices.size();

    return common::Classification(name(), nInliers/nPoints);
}

void ModelFitting::visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients& coefficients)
{
    switch(_model) {
    case pcl::SACMODEL_SPHERE:
    {
        viewer.addSphere(coefficients,"Sphere");
        break;
    }
    case pcl::SACMODEL_CYLINDER:
    {
        viewer.addCylinder(coefficients,"Cylinder");
        break;
    }
    default:
    {
        std::cerr << "Undefined model" << std::endl;
        break;
    }
    }
}
