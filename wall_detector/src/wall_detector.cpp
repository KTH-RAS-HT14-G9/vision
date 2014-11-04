#include <ros/ros.h>
#include "wall_extractor.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <wall_detector/Walls.h>

//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

WallExtractor _extractor;
WallExtractor::SharedPointCloud _pcloud;

double _distance_threshold = 0.01; std::string _distance_threshold_key = "/vision/walls/distThresh";
double _leaf_size = 0.02; std::string _leaf_size_key = "/vision/walls/leafSize";
double _halt_condition = 0.2; std::string _halt_condition_key = "/vision/walls/haltCondition";

//------------------------------------------------------------------------------
// Callbacks

void callback_point_cloud(const WallExtractor::SharedPointCloud& pcloud)
{
    _pcloud = pcloud;
}

//------------------------------------------------------------------------------
// Test cases

WallExtractor::PointCloud::Ptr testcase() {

    WallExtractor::PointCloud::Ptr cloud(new WallExtractor::PointCloud);

    // Fill in the cloud data
    cloud->width  = 150+300+150+100;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // Generate the data
    int k = 0;
    // Generate bottom
    for (size_t i = 0; i < 300; ++i)
    {
        cloud->points[i].x = 60 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 30 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 0.2 * rand () / (RAND_MAX + 1.0f);
    }
    k+=300;

    // Generate left wall
    for (size_t i = 0; i < 150; ++i)
    {
        cloud->points[k+i].x = 60 * rand () / (RAND_MAX + 1.0f);
        cloud->points[k+i].y = -1;
        cloud->points[k+i].z = +1 + 20 * rand () / (RAND_MAX + 1.0f);
    }
    k+=150;

    // Generate right wall
    for (size_t i = 0; i < 150; ++i)
    {
        cloud->points[k+i].x = 60 * rand () / (RAND_MAX + 1.0f);
        cloud->points[k+i].y = +1;
        cloud->points[k+i].z = +1 + 20 * rand () / (RAND_MAX + 1.0f);
    }
    k+=150;

    // Generate front wall
    for (size_t i = 0; i < 100; ++i)
    {
        cloud->points[k+i].x = +1;
        cloud->points[k+i].y = 20 * rand () / (RAND_MAX + 1.0f);
        cloud->points[k+i].z = +1 + 20 * rand () / (RAND_MAX + 1.0f);
    }

    return cloud;
}

//------------------------------------------------------------------------------
// Message converter

template<typename T>
void append_all(std::vector<T>& a, const std::vector<T>& b)
{
    a.insert(a.begin(), b.begin(), b.end());
}

typedef boost::shared_ptr<wall_detector::Walls> WallsMsgPtr;
WallsMsgPtr generate_walls_msg(const WallExtractor::WallsPtr& data)
{
    WallsMsgPtr msg(new wall_detector::Walls);

    for(int i = 0; i < data->size(); ++i)
    {
        wall_detector::Wall wall;
        pcl::ModelCoefficientsConstPtr coeff = data->at(i).get_coefficients();
        pcl::PointIndicesConstPtr inliers = data->at(i).get_inliers();

        append_all<float>(wall.plane_coefficients, coeff->values);
        append_all<int>(wall.point_cloud_inliers, inliers->indices);

        msg->walls.push_back(wall);
    }

    return msg;
}

//------------------------------------------------------------------------------
// Entry point

int main(int argc, char **argv)
{
//    _pcloud = testcase();
//    return 0;

    ros::init(argc, argv, "wall_detector");

    ros::NodeHandle n;
    n.setParam(_distance_threshold_key, _distance_threshold);
    n.setParam(_leaf_size_key, _leaf_size);
    n.setParam(_halt_condition_key, _halt_condition);

    Eigen::Vector3d leaf_size(_leaf_size,_leaf_size,_leaf_size);

    ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >
            ("/camera/point_cloud", 3, callback_point_cloud);

    ros::Publisher pub_walls = n.advertise<wall_detector::Walls>("/vision/walls",10);

    ros::Rate rate(PUBLISH_FREQUENCY);

    while(n.ok())
    {
        n.getParamCached(_distance_threshold_key, _distance_threshold);
        n.getParamCached(_leaf_size_key, _leaf_size);
        n.getParamCached(_halt_condition_key, _halt_condition);

        leaf_size.setConstant(_leaf_size);

        WallExtractor::WallsPtr walls = _extractor.extract(_pcloud, _distance_threshold, _halt_condition, leaf_size);

        pub_walls.publish(generate_walls_msg(walls));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
