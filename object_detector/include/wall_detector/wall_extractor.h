#ifndef WALL_EXTRACTOR_H
#define WALL_EXTRACTOR_H

#include <common/debug.h>
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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

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

    void set_down_vector(const Eigen::Vector3f& down);

    common::vision::SegmentedPlane::ArrayPtr extract(
            const common::SharedPointCloudRGB &cloud,
            double distance_threshold,
            double halt_condition,
            int downsample_target_n,
            double samples_max_dist);

protected:

    int find_ground_plane(common::vision::SegmentedPlane::ArrayPtr& walls);
    void determine_XY_bounding_box(const pcl::ModelCoefficients& plane,
                                   const pcl::PointCloud<pcl::PointXYZRGB>& points,
                                   const std::vector<int>& indices,
                                   common::OrientedBoundingBox& obb);

    pcl::SACSegmentation<pcl::PointXYZRGB> _seg;
    common::PointCloudRGB::Ptr _cloud_filtered, _cloud_p, _cloud_f;
    pcl::ExtractIndices<pcl::PointXYZRGB> _extract;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> _cluster_ex;
    pcl::search::Search<pcl::PointXYZRGB>::Ptr _kd_tree;

    Eigen::Vector3f _down;
private:
#if ENABLE_VISUALIZATION_PLANES==1
    pcl::visualization::PCLVisualizer _viewer;
    pcl::ProjectInliers<pcl::PointXYZRGB> _proj;
    pcl::ConvexHull<pcl::PointXYZRGB> _hull;
    common::Colors _colors;
#endif
};

#endif // WALL_EXTRACTOR_H
