#ifndef SHAPE_FITTING_H
#define SHAPE_FITTING_H

#include <pcl/segmentation/sac_segmentation.h>
#include <common/types.h>

class ShapeFitting
{
public:
    ShapeFitting(pcl::SacModel model);

    double classify(const common::SharedPointCloudRGB& cloud);

protected:
    pcl::PointIndices _inliers;
    pcl::ModelCoefficients _coefficients;
    pcl::SACSegmentation<pcl::PointXYZRGB> _seg;
};

#endif // SHAPE_FITTING_H
