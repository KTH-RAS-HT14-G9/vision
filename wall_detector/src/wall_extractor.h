#ifndef WALL_EXTRACTOR_H
#define WALL_EXTRACTOR_H

#define ENABLE_VISUALIZATION 0

#include "wall_detector/segmented_wall.h"

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

/*class Wall {
public:
    Wall(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& original,
         const pcl::ModelCoefficientsConstPtr& coefficients,
         const pcl::PointIndicesConstPtr& inliers)
    {
        _original_cloud = original;
        _coefficients = coefficients;
        _inliers = inliers;
    }

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr get_original_cloud() { return _original_cloud; }
    pcl::ModelCoefficientsConstPtr get_coefficients() { return _coefficients; }
    pcl::PointIndicesConstPtr get_inliers() { return _inliers; }

protected:
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr _original_cloud;
    pcl::ModelCoefficientsConstPtr _coefficients;
    pcl::PointIndicesConstPtr _inliers;
};*/

class WallExtractor
{
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::ConstPtr SharedPointCloud;

    typedef boost::shared_ptr<std::vector<SegmentedWall> > WallsPtr;

    WallExtractor();

    void set_frustum_culling(float near_plane_dist,
                             float far_plane_dist,
                             float horizontal_fov,
                             float vertical_fov);

    WallsPtr extract(const SharedPointCloud &cloud,
                     double distance_threshold,
                     double halt_condition,
                     const Eigen::Vector3d& voxel_leaf_size);

protected:
    pcl::FrustumCulling<pcl::PointXYZ> _frustum;

    pcl::SACSegmentation<pcl::PointXYZ> _seg;
    PointCloud::Ptr _cloud_filtered, _cloud_p, _cloud_f;
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

    static void AddPCL(pcl::visualization::PCLVisualizer& vis, const SharedPointCloud& cloud, const std::string& key, int r, int g, int b)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud,r,g,b);
        WallExtractor::AddPCL(vis,cloud,key,color);
    }

    static void AddPCL(pcl::visualization::PCLVisualizer& vis, const SharedPointCloud& cloud, const std::string& key)
    {
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> color(cloud);
        WallExtractor::AddPCL(vis,cloud,key,color);
    }

    static void AddPCL(pcl::visualization::PCLVisualizer& vis,
                const SharedPointCloud& cloud,
                const std::string& key,
                pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>& color)
    {
        vis.addPointCloud<pcl::PointXYZ>(cloud, color, key);
        vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, key);
    }
#endif
};

WallExtractor::WallExtractor() {

    set_frustum_culling(0.01f, 10.0f, 60.0f, 45.0f);

    // Optional
    _seg.setOptimizeCoefficients (true);
    // Mandatory
    _seg.setModelType (pcl::SACMODEL_PLANE);
    _seg.setMethodType(pcl::SAC_RANSAC);

    _cloud_filtered = PointCloud::Ptr(new PointCloud);
    _cloud_p = PointCloud::Ptr(new PointCloud);
    _cloud_f = PointCloud::Ptr(new PointCloud);


#if ENABLE_VISUALIZATION==1
    //visualization::PCLVisualizer viewer("Viewer");
    _viewer.addCoordinateSystem (1.0);
    _viewer.initCameraParameters ();

    _proj.setModelType(pcl::SACMODEL_PLANE);

    _colors.push_back(WallExtractor::Color(255,0,0));
    _colors.push_back(WallExtractor::Color(0,255,0));
    _colors.push_back(WallExtractor::Color(0,0,255));
    _colors.push_back(WallExtractor::Color(255,255,0));
    _colors.push_back(WallExtractor::Color(0,255,255));
#endif
}

void WallExtractor::set_frustum_culling(float near_plane_dist,
                                        float far_plane_dist,
                                        float horizontal_fov,
                                        float vertical_fov)
{
    _frustum.setNearPlaneDistance(near_plane_dist);
    _frustum.setFarPlaneDistance(far_plane_dist);
    _frustum.setHorizontalFOV(horizontal_fov);
    _frustum.setVerticalFOV(vertical_fov);
}

/**
  * Adapted from http://pointclouds.org/documentation/tutorials/extract_indices.php
  *
  * Uses RANSAC to find plane models in point cloud.
  * Finds walls in an iterative fashion, where the walls with strongest consense
  * are sequentially removed from the whole point cloud until there are only
  * @see(halt_condition) percent of the original points left.
  *
  * @param distance_threshold   Minimum distance to the detected wall
  * @param halt_condition       Halting condition defined in percent of the original
  *                             number of points in the cloud. Is the condition
  *                             reached, the method is stopping to find any further
  *                             planes.
  * @param voxel_leaf_size      Defines voxel size. The greater the voxels the faster
  *                             the computation and the worse the accuracy.
  * @return Shared pointer of array of @link(WallExtractor::Wall). Empty if no
  *         walls found.
  */
WallExtractor::WallsPtr WallExtractor::extract(const SharedPointCloud &cloud,
                                        double distance_threshold = 0.01,
                                        double halt_condition = 0.1,
                                        const Eigen::Vector3d& voxel_leaf_size = Eigen::Vector3d(0.1,0.1,0.1))
{
    using namespace pcl;

    _cloud_filtered->clear();
    _cloud_p->clear();
    _cloud_f->clear();

#if ENABLE_VISUALIZATION==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();
#endif

    //Filter all points in frustum
    _frustum.setInputCloud(cloud);
    _frustum.filter(*_cloud_f);

    // Create the filtering object: downsample the dataset using the given leaf size
    _downsampler.setInputCloud (_cloud_f);
    _downsampler.setLeafSize (voxel_leaf_size(0), voxel_leaf_size(1), voxel_leaf_size(2));
    _downsampler.filter (*_cloud_filtered);
    _cloud_f->clear();


    WallExtractor::WallsPtr walls(new std::vector<SegmentedWall>);

    _seg.setDistanceThreshold (distance_threshold);


    int i = 0, nr_points = (int) _cloud_filtered->points.size ();

    // While halt_condition% of the original cloud is still there
    while (_cloud_filtered->points.size () > halt_condition * nr_points)
    {
        ModelCoefficientsPtr coefficients(new ModelCoefficients);
        PointIndicesPtr inliers(new PointIndices);

        // Segment the largest planar component from the remaining cloud
        _seg.setInputCloud (_cloud_filtered);
        _seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        //add wall to result set
        walls->push_back(SegmentedWall(coefficients,inliers));

        // Extract the inliers
        _extract.setInputCloud (_cloud_filtered);
        _extract.setIndices (inliers);
        _extract.setNegative (false);
        _extract.filter (*_cloud_p);

#if ENABLE_VISUALIZATION==1
        std::stringstream ss;
        ss << "Plane: " << i;
        if (i < _colors.size())
            WallExtractor::AddPCL(_viewer, _cloud_p, ss.str(), _colors[i].r, _colors[i].g, _colors[i].b);
        else
            WallExtractor::AddPCL(_viewer, _cloud_p, ss.str());

        //Eigen::Vector4d centroid;
        //compute3DCentroid<PointXYZ>(*_cloud_p, centroid);
        //_viewer.addPlane(*coefficients,centroid(0),centroid(1),centroid(2),ss.str());

        PointCloud::Ptr cloud_projected = PointCloud::Ptr(new PointCloud);

        _proj.setInputCloud (_cloud_p);
        _proj.setModelCoefficients (coefficients);
        _proj.filter (*cloud_projected);

        PointCloud::Ptr cloud_hull = PointCloud::Ptr(new PointCloud);
        _hull.setInputCloud(cloud_projected);
        _hull.reconstruct(*cloud_hull);
        _hull.setDimension(2);

        _viewer.addPolygon<PointXYZ>(cloud_hull, _colors[i].r, _colors[i].g, _colors[i].b, ss.str());
#endif

        // Create the filtering object
        _extract.setNegative (true);
        _extract.filter (*_cloud_f);
        _cloud_filtered.swap (_cloud_f);
        i++;
    }

#if ENABLE_VISUALIZATION==1
    std::cerr << "Walls fitted: " << i << std::endl;

    WallExtractor::AddPCL(_viewer,_cloud_filtered,"unclassified",255,255,255);

    //viewer.resetCamera();
    _viewer.spinOnce(100);
#endif

    return walls;
}

#endif // WALL_EXTRACTOR_H
