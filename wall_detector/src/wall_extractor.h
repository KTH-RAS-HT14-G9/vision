#ifndef WALL_EXTRACTOR_H
#define WALL_EXTRACTOR_H

#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Core>

#include <pcl/visualization/cloud_viewer.h>

class WallExtractor
{
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::ConstPtr SharedPointCloud;

    WallExtractor();

    Eigen::MatrixX4d extract(const SharedPointCloud &cloud,
                             double distance_threshold,
                             const Eigen::Vector3d& voxel_leaf_size);

protected:
    pcl::SACSegmentation<pcl::PointXYZ> _seg;
    PointCloud::Ptr _cloud_filtered_downsampled;
    PointCloud::Ptr _cloud_filtered, _cloud_p, _cloud_f;
    pcl::ExtractIndices<pcl::PointXYZ> _extract;
    pcl::VoxelGrid<pcl::PointXYZ> _downsampler;
};

WallExtractor::WallExtractor() {
    // Optional
    _seg.setOptimizeCoefficients (true);
    // Mandatory
    _seg.setModelType (pcl::SACMODEL_PLANE);
    _seg.setMethodType(pcl::SAC_RANSAC);

    _cloud_filtered_downsampled = PointCloud::Ptr(new PointCloud);
    _cloud_filtered = PointCloud::Ptr(new PointCloud);
    _cloud_p = PointCloud::Ptr(new PointCloud);
    _cloud_f = PointCloud::Ptr(new PointCloud);
}

Eigen::MatrixX4d WallExtractor::extract(const SharedPointCloud &cloud,
                                        double distance_threshold = 0.01,
                                        const Eigen::Vector3d& voxel_leaf_size = Eigen::Vector3d(0.1,0.1,0.1))
{
    using namespace pcl;

    _cloud_filtered_downsampled->clear();
    _cloud_filtered->clear();
    _cloud_p->clear();
    _cloud_f->clear();

    // Create the filtering object: downsample the dataset using the given leaf size
    _downsampler.setInputCloud (cloud);
    _downsampler.setLeafSize (voxel_leaf_size(0), voxel_leaf_size(1), voxel_leaf_size(2));
    _downsampler.filter (*_cloud_filtered_downsampled);


    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);

    _seg.setDistanceThreshold (distance_threshold);


    int i = 0, nr_points = (int) _cloud_filtered->points.size ();

    // While 30% of the original cloud is still there
    while (_cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        _seg.setInputCloud (_cloud_filtered);
        _seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        _extract.setInputCloud (_cloud_filtered);
        _extract.setIndices (inliers);
        _extract.setNegative (false);
        _extract.filter (*_cloud_p);
        std::cerr << "PointCloud representing the planar component: " << _cloud_p->width * _cloud_p->height << " data points." << std::endl;

        // Create the filtering object
        _extract.setNegative (true);
        _extract.filter (*_cloud_f);
        _cloud_filtered.swap (_cloud_f);
        i++;
    }

//    visualization::CloudViewer viewer ("Simple Cloud Viewer");
//    pcl::PointCloud<pcl::PointXYZ> plane;

//    viewer.showCloud (plane);
//    while (!viewer.wasStopped ())
//    {
//    }

    return Eigen::MatrixX4d();
}

#endif // WALL_EXTRACTOR_H
