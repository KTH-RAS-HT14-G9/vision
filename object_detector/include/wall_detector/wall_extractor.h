#ifndef WALL_EXTRACTOR_H
#define WALL_EXTRACTOR_H

#define ENABLE_VISUALIZATION 1

#include "wall_detector/segmented_wall.h"
#include <common/types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/frustum_culling.h>

#include <Eigen/Core>

#if ENABLE_VISUALIZATION==1
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
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

    void set_camera_matrix(const Eigen::Matrix4f& m);

    SegmentedWall::ArrayPtr extract(const common::SharedPointCloud &cloud,
                     double distance_threshold,
                     double halt_condition,
                     const Eigen::Vector3d& voxel_leaf_size);

protected:
    pcl::FrustumCulling<pcl::PointXYZ> _frustum;

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
#if ENABLE_VISUALIZATION==1
    pcl::visualization::PCLVisualizer _viewer;
    pcl::ProjectInliers<pcl::PointXYZ> _proj;
    pcl::ConvexHull<pcl::PointXYZ> _hull;
    std::vector<Color> _colors;

    static void AddPCL(pcl::visualization::PCLVisualizer& vis, const common::SharedPointCloud& cloud, const std::string& key, int r, int g, int b)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud,r,g,b);
        WallExtractor::AddPCL(vis,cloud,key,color);
    }

    static void AddPCL(pcl::visualization::PCLVisualizer& vis, const common::SharedPointCloud& cloud, const std::string& key)
    {
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color(cloud);
        WallExtractor::AddPCL(vis,cloud,key,color);
    }

    static void AddPCL(pcl::visualization::PCLVisualizer& vis,
                const common::SharedPointCloud& cloud,
                const std::string& key,
                pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>& color)
    {
        vis.addPointCloud<pcl::PointXYZ>(cloud, color, key);
        vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, key);
    }
#endif
};

#endif // WALL_EXTRACTOR_H
