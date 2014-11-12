#ifndef SHAPE_FITTING_H
#define SHAPE_FITTING_H

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <common/types.h>

class ShapeFitting
{
public:
    ShapeFitting(pcl::SacModel model);

    double classify(const common::SharedPointCloudRGB& cloud, pcl::ModelCoefficients& coefficients);

    void set_segmentation(double dist_thresh, double normal_dist_weight, double min_r, double max_r);
    void set_normal_estimation(int k);

protected:
    pcl::PointIndices _inliers;

    //pcl::SACSegmentation<pcl::PointXYZRGB> _seg;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> _seg;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> _normal_est;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr _kd_tree;
    pcl::PointCloud<pcl::Normal>::Ptr _normals;
};

#endif // SHAPE_FITTING_H
