#ifndef ROI_EXTRACTOR_H
#define ROI_EXTRACTOR_H

#define ENABLE_VISUALIZATION_ROIS 1

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <wall_detector/Walls.h> //TODO: replace with ros independent class
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <common/types.h>
#include <common/roi.h>
#include <common/segmented_plane.h>

#include <pcl/filters/plane_clipper3D.h>
#include <pcl/segmentation/extract_clusters.h>

#if ENABLE_VISUALIZATION_ROIS==1
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

class ROIExtractor
{
public:

    ROIExtractor();
    ~ROIExtractor();

    void set_rgb_image(const cv_bridge::CvImageConstPtr& img);

    common::vision::ROIArrayPtr extract(const common::vision::SegmentedPlane::ArrayPtr& walls,
                                        const common::SharedPointCloudRGB& pcloud,
                                        double wall_thickness);

    void set_cluster_constraints(double cluster_tolerance,
                                 int min_cluster_size,
                                 int max_cluster_size);

protected:

    //void filter_points_on_plane()

    cv_bridge::CvImageConstPtr _rgb;

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> _cluster_ex;
    pcl::PlaneClipper3D<pcl::PointXYZRGB> _plane_clipper;

private:
#if ENABLE_VISUALIZATION_ROIS==1
    pcl::visualization::PCLVisualizer _viewer;
    //std::vector<Color> _colors;
#endif
};

#endif // ROI_EXTRACTOR_H
