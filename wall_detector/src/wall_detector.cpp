#include <ros/ros.h>
#include "wall_extractor.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

WallExtractor _extractor;
WallExtractor::SharedPointCloud _pcloud;

double _distance_threshold = 0.1; std::string _distance_threshold_key = "/vision/walls/distThresh";

//------------------------------------------------------------------------------
// Callbacks

void callback_point_cloud(const WallExtractor::SharedPointCloud& pcloud)
{
    _pcloud = pcloud;
}

//------------------------------------------------------------------------------
// Entry point

void testcase() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width  = 150+300+150;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // Generate the data
    int k = 0;
    // Generate bottom
    for (size_t i = 0; i < 300; ++i)
    {
        cloud->points[i].x = 60 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 30 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 0.0;
    }
    k+=300;

    // Generate left wall
    for (size_t i = 0; i < 150; ++i)
    {
        cloud->points[k+i].x = 60 * rand () / (RAND_MAX + 1.0f);
        cloud->points[k+i].y = -1;
        cloud->points[k+i].z = 20 * rand () / (RAND_MAX + 1.0f);
    }
    k+=150;

    // Generate right wall
    for (size_t i = 0; i < 150; ++i)
    {
        cloud->points[k+i].x = 60 * rand () / (RAND_MAX + 1.0f);
        cloud->points[k+i].y = +1;
        cloud->points[k+i].z = 20 * rand () / (RAND_MAX + 1.0f);
    }

    _extractor.extract(cloud);
}

int main(int argc, char **argv)
{
    testcase();
    return 0;

    ros::init(argc, argv, "wall_detector");

    ros::NodeHandle n;
    n.setParam(_distance_threshold_key, _distance_threshold);

    ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >
            ("/camera/point_cloud", 3, callback_point_cloud);

    ros::Rate rate(PUBLISH_FREQUENCY);

    while(n.ok())
    {
        n.getParamCached(_distance_threshold_key,_distance_threshold);

        _extractor.extract(_pcloud, _distance_threshold);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
