#ifndef SEGMENTED_WALL_H
#define SEGMENTED_WALL_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

class SegmentedWall {
public:
    SegmentedWall(const pcl::ModelCoefficientsConstPtr& coefficients,
                  const pcl::PointIndicesConstPtr& inliers)
    {
        _coefficients = coefficients;
        _inliers = inliers;
    }

    pcl::ModelCoefficientsConstPtr get_coefficients() { return _coefficients; }
    pcl::PointIndicesConstPtr get_inliers() { return _inliers; }

protected:
    pcl::ModelCoefficientsConstPtr _coefficients;
    pcl::PointIndicesConstPtr _inliers;
};

#endif // WALLS_H
