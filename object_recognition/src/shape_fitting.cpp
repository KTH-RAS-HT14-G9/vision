#include "shape_fitting/shape_fitting.h"

ShapeFitting::ShapeFitting(pcl::SacModel model)
{
    // Optional
    _seg.setOptimizeCoefficients (true);
    // Mandatory
    _seg.setModelType (model);
    _seg.setMethodType(pcl::SAC_RANSAC);

    _normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    _kd_tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
}

void ShapeFitting::set_segmentation(double dist_thresh, double normal_dist_weight, double min_r, double max_r)
{
    _seg.setDistanceThreshold(dist_thresh);
    _seg.setNormalDistanceWeight(normal_dist_weight);
    _seg.setRadiusLimits(min_r,max_r);
}

void ShapeFitting::set_normal_estimation(int k)
{
    _normal_est.setKSearch(k);
}

double ShapeFitting::classify(const common::SharedPointCloudRGB &cloud, pcl::ModelCoefficients &coefficients)
{
    _normals->clear();
    _inliers.indices.clear();
    coefficients.values.clear();

    _normal_est.setInputCloud(cloud);
    _normal_est.setSearchMethod(_kd_tree);
    _normal_est.compute(*_normals);

    _seg.setInputCloud(cloud);
    _seg.setInputNormals(_normals);
    _seg.segment(_inliers,coefficients);

    double nPoints = (double)cloud->size();
    double nInliers = (double)_inliers.indices.size();

    return nInliers/nPoints;
}
