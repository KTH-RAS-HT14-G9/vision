#include "wall_detector/wall_extractor.h"
#include <pre_filter/pre_filter.h>
#include <common/types.h>

WallExtractor::WallExtractor()
    :_down(0,-1,0)
{

    using namespace common;


    _kd_tree = pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>(true));

    // Optional
    _seg.setOptimizeCoefficients (true);
    // Mandatory
    _seg.setModelType (pcl::SACMODEL_PLANE);
    _seg.setMethodType(pcl::SAC_RANSAC);

    _cloud_filtered = PointCloudRGB::Ptr(new PointCloudRGB);
    _cloud_p = PointCloudRGB::Ptr(new PointCloudRGB);
    _cloud_f = PointCloudRGB::Ptr(new PointCloudRGB);

#if ENABLE_VISUALIZATION_PLANES==1
    //visualization::PCLVisualizer viewer("Viewer");
    _viewer.addCoordinateSystem (1.0);
    _viewer.initCameraParameters ();
    //_viewer.setBackgroundColor(255,255,255,0);

    _proj.setModelType(pcl::SACMODEL_PLANE);
#endif
}

void WallExtractor::set_down_vector(const Eigen::Vector3f &down)
{
    _down = down;
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
common::vision::SegmentedPlane::ArrayPtr WallExtractor::extract(const common::SharedPointCloudRGB &cloud,
                                        double distance_threshold = 0.01,
                                        double halt_condition = 0.1,
                                        int downsample_target_n = 5000,
                                        double samples_max_dist = 0.2)
{
    using pcl::ModelCoefficients;
    using pcl::PointIndices;

    using common::PointCloudRGB;
    using common::SharedPointCloudRGB;
    using common::vision::SegmentedPlane;




    _cloud_filtered->clear();
    _cloud_p->clear();
    _cloud_f->clear();

#if ENABLE_VISUALIZATION_PLANES==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();
    _colors.reset();
#endif

    PointIndices::Ptr inliers(new PointIndices);
    inliers->indices.resize(cloud->size());
    for(int i = 0; i < cloud->size(); ++i) inliers->indices[i] = i;

    PreFilter::fast_downsampling(cloud,inliers,_cloud_filtered,downsample_target_n);
    //pcl::copyPointCloud(*cloud,*_cloud_filtered);

    SegmentedPlane::ArrayPtr walls(new std::vector<SegmentedPlane>);

    //_seg.setSamplesMaxDist(samples_max_dist, _kd_tree);
    _seg.setDistanceThreshold (distance_threshold);

    int i = 0, nr_points = (int) _cloud_filtered->points.size ();

    // While halt_condition% of the original cloud is still there
    while (_cloud_filtered->points.size () > halt_condition * nr_points)
    {
        ModelCoefficients::Ptr coefficients(new ModelCoefficients);
        inliers->indices.clear();

        // Segment the largest planar component from the remaining cloud
        _seg.setInputCloud (_cloud_filtered);
        _kd_tree->setInputCloud(_cloud_filtered);
        _seg.setSamplesMaxDist(samples_max_dist, _kd_tree);
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

        //add wall to result set
        common::OrientedBoundingBox obb(_cloud_p);
        walls->push_back(SegmentedPlane(coefficients,obb));

#if ENABLE_VISUALIZATION_PLANES==1
        std::stringstream ss;
        ss << "Plane: " << i;
        common::Color c = _colors.next();
        pcl::visualization::AddPointCloud(_viewer, _cloud_p, ss.str(), c.r, c.g, c.b);

        Eigen::Vector4d centroid;
        pcl::compute3DCentroid<pcl::PointXYZRGB>(*_cloud_p, centroid);
        _viewer.addPlane(*coefficients,centroid(0),centroid(1),centroid(2),ss.str());

        PointCloudRGB::Ptr cloud_projected = PointCloudRGB::Ptr(new PointCloudRGB);

        _proj.setInputCloud (_cloud_p);
        _proj.setModelCoefficients (coefficients);
        _proj.filter (*cloud_projected);

        PointCloudRGB::Ptr cloud_hull = PointCloudRGB::Ptr(new PointCloudRGB);
        _hull.setInputCloud(cloud_projected);
        _hull.reconstruct(*cloud_hull);
        _hull.setDimension(2);

        _viewer.addPolygon<pcl::PointXYZRGB>(cloud_hull, c.r, c.g, c.b, ss.str());

        ss << "_obb";
        _viewer.addCube(obb.get_translation(), obb.get_rotation(), obb.get_width(), obb.get_height(), obb.get_depth(), ss.str());
#endif

        // Create the filtering object
        _extract.setNegative (true);
        _extract.filter (*_cloud_f);
        _cloud_filtered.swap (_cloud_f);
        i++;
    }

#if ENABLE_VISUALIZATION_PLANES==1
    std::cerr << "Walls fitted: " << i << std::endl;

    pcl::visualization::AddPointCloud(_viewer,_cloud_filtered,"unclassified",255,255,255);

    //viewer.resetCamera();
    _viewer.spinOnce(1,true);
#endif

    int ground_plane = find_ground_plane(walls);
    //place ground plane in the first index
    if (ground_plane >= 0) {
        if (ground_plane > 0)
            std::swap( walls->at(0), walls->at(ground_plane) );

        walls->at(0).set_as_ground_plane();
    }

    return walls;
}

int WallExtractor::find_ground_plane(common::vision::SegmentedPlane::ArrayPtr& walls)
{
    //find ground plane
    float max_metric = 0.0;
    int ground_plane = 0;
    for(int i = 0; i < walls->size(); ++i)
    {
        const pcl::ModelCoefficientsConstPtr& plane = walls->at(i).get_coefficients();
        Eigen::Vector3f normal(plane->values[0],plane->values[1],plane->values[2]);
        normal.normalize();

        float metric = std::abs(_down.dot(normal));
        if (metric > max_metric) {
            max_metric = metric;
            ground_plane = i;
        }

        //std::cerr << "Wall " << i << ", Metric: " << metric << std::endl;
    }

    //check if it's the true ground (happens only when we do not see the ground)
    if (max_metric < 0.8) {
        ground_plane = -1;
        std::cerr << "-----------> No ground to see <-----------" << std::endl;
        std::cerr << "Greatest metric: " << max_metric << std::endl << std::endl;

        return -1;
    }

    return ground_plane;
}
