#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <wall_detector/Walls.h> //TODO: replace with ros independent class
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <wall_detector/segmented_wall.h>


class ROIExtractor
{
public:

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::ConstPtr SharedPointCloud;

    typedef struct anomaly_blob_s {
        float depth_centroid;
        cv::Rect roi;
        std::vector<std::pair<int, double> > color_probabilities;
    } anomaly_blob;

    ROIExtractor();

    void set_walls(const wall_detector::WallsConstPtr& walls);
    void set_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void set_rgb_image(const cv_bridge::CvImageConstPtr& img);
    void set_depth_image(const cv_bridge::CvImageConstPtr& img);

    std::vector<ROIExtractor::anomaly_blob> detect(const SegmentedWall::ArrayPtr& walls, const SharedPointCloud& pcloud);

protected:
    cv_bridge::CvImageConstPtr _rgb;
    cv_bridge::CvImageConstPtr _depth;
    cv::Mat _segmented;
};

ROIExtractor::ROIExtractor() {
    _segmented = cv::Mat(480, 640, CV_8UC1);
}

void ROIExtractor::set_rgb_image(const cv_bridge::CvImageConstPtr& img)
{
    //create new _segmented, if the sizes do not match
    if (_segmented.size[0] != img->image.size[0] || _segmented.size[0] != img->image.size[1])
        _segmented = cv::Mat(img->image.size[0], img->image.size[1], CV_8UC1);

    _rgb = img;
}

void ROIExtractor::set_depth_image(const cv_bridge::CvImageConstPtr& img)
{
    _depth = img;
}

std::vector<ROIExtractor::anomaly_blob> ROIExtractor::detect(const SegmentedWall::ArrayPtr& walls, const SharedPointCloud& pcloud)
{
    using namespace Eigen;

    cv::watershed(_rgb->image, _segmented);

    return std::vector<ROIExtractor::anomaly_blob>();
}

#endif // OBSTACLE_DETECTOR_H
