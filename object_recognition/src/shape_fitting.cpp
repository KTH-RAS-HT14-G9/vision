#include "shape_fitting/shape_fitting.h"

ShapeFitting::ShapeFitting(pcl::SacModel model)
{
    // Optional
    _seg.setOptimizeCoefficients (true);
    // Mandatory
    _seg.setModelType (model);
    _seg.setMethodType(pcl::SAC_RANSAC);
}

double ShapeFitting::classify(const common::SharedPointCloudRGB& cloud)
{
    _seg.setInputCloud(cloud);
    _seg.segment(_inliers,_coefficients);

    double nPoints = (double)cloud->size();
    double nInliers = (double)_inliers.indices.size();

    return nInliers/nPoints;
}
