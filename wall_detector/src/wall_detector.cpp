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

int main(int argc, char **argv)
{
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
