#ifndef WALL_EXTRACTOR_H
#define WALL_EXTRACTOR_H

#define ENABLE_VISUALIZATION_PLANES 0

#include <common/segmented_plane.h>
#include <common/types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Eigen/Core>

#if ENABLE_VISUALIZATION_PLANES==1
#include <common/visualization_addons.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#endif

/**
  * Thoughts:
  * - Discard planes with too few supporting points
  * - Only take 5 strongest planes
  */

class WallExtractor
{
public:

    WallExtractor();

    void set_frustum_culling(float near_plane_dist,
                             float far_plane_dist,
                             float horizontal_fov,
                             float vertical_fov);

    void set_outlier_removal(int meanK, double stddev_multhresh);

    void set_camera_matrix(const Eigen::Matrix4f& m);

    common::vision::SegmentedPlane::ArrayPtr extract(
            const common::SharedPointCloud &cloud,
            double distance_threshold,
            double halt_condition,
            const Eigen::Vector3d& voxel_leaf_size);

protected:
    pcl::FrustumCulling<pcl::PointXYZ> _frustum;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> _sor;

    pcl::SACSegmentation<pcl::PointXYZ> _seg;
    common::PointCloud::Ptr _cloud_filtered, _cloud_p, _cloud_f;
    pcl::ExtractIndices<pcl::PointXYZ> _extract;
    pcl::VoxelGrid<pcl::PointXYZ> _downsampler;

    class Color {
    public:
        Color(int cr, int cg, int cb)
            :r(cr),g(cg),b(cb) {}
        int r, g, b;
    };

private:
#if ENABLE_VISUALIZATION_PLANES==1
    pcl::visualization::PCLVisualizer _viewer;
    pcl::ProjectInliers<pcl::PointXYZ> _proj;
    pcl::ConvexHull<pcl::PointXYZ> _hull;
    std::vector<Color> _colors;
#endif
};

#endif // WALL_EXTRACTOR_H
