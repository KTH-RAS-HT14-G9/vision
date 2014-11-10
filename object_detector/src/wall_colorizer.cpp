#include <ros/ros.h>
#include "wall_detector/wall_extractor.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <wall_detector/Walls.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Core>
#include <common/parameter.h>

//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

WallExtractor _wall_extractor;
common::SharedPointCloud _pcloud;
Eigen::Matrix4f _camera_matrix;

Parameter<double> _distance_threshold("/vision/walls/dist_thres", 0.01);
Parameter<double> _leaf_size("/vision/walls/leaf_size", 0.02);
Parameter<double> _halt_condition("/vision/walls/halt_condition", 0.2);

Parameter<double> _frustum_near("/vision/walls/frustum/near", 0.3);
Parameter<double> _frustum_far("/vision/walls/frustum/far", 1.7);
Parameter<double> _frustum_horz_fov("/vision/walls/frustum/horz_fov", 60.0);
Parameter<double> _frustum_vert_fov("/vision/walls/frustum/vert_fov", 50.0);

Parameter<int>    _outlier_meanK("/vision/walls/outliers/meanK", 50);
Parameter<double> _outlier_thresh("/vision/walls/outliers/thresh", 0.5);

//------------------------------------------------------------------------------
// Callbacks

void callback_point_cloud(const common::SharedPointCloud& pcloud)
{
    _pcloud = pcloud;
}

void callback_camera_matrix(const sensor_msgs::CameraInfoConstPtr& m)
{
    _camera_matrix <<   m->D[0], m->D[1], m->D[2], m->D[3],
                        m->K[0], m->K[1], m->K[2], m->K[3],
                        m->R[0], m->R[1], m->R[2], m->R[3],
                        m->P[0], m->P[1], m->P[2], m->P[3];

    _wall_extractor.set_camera_matrix(_camera_matrix);
}

//------------------------------------------------------------------------------
// Test cases

common::PointCloud::Ptr testcase() {

    common::PointCloud::Ptr cloud(new common::PointCloud);

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
// Entry point

int main(int argc, char **argv)
{
//    _pcloud = testcase();
//    return 0;

    ros::init(argc, argv, "wall_colorizer");

    ros::NodeHandle n;

    Eigen::Vector3d leaf_size(_leaf_size(),_leaf_size(),_leaf_size());

    ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >
            ("/camera/depth/points", 3, callback_point_cloud);
//    ros::Subscriber sub_camera_m = n.subscribe<sensor_msgs::CameraInfo>
//            ("/camera/ir/camera_info", 1, callback_camera_matrix);

    ros::Publisher pub_walls = n.advertise<wall_detector::Walls>("/vision/walls",10);

    ros::Rate rate(PUBLISH_FREQUENCY);

    while(n.ok())
    {
        leaf_size.setConstant(_leaf_size());
        _wall_extractor.set_frustum_culling(_frustum_near(), _frustum_far(), _frustum_horz_fov(), _frustum_vert_fov());
        _wall_extractor.set_outlier_removal(_outlier_meanK(), _outlier_thresh());

        ros::Time time = ros::Time::now();

        common::vision::SegmentedPlane::ArrayPtr walls = _wall_extractor.extract(_pcloud, _distance_threshold(), _halt_condition(), leaf_size);

        ROS_INFO("Time spent on extracting planes: %lf\n", (ros::Time::now().toSec() - time.toSec()));

        pub_walls.publish(common::vision::segmentedPlaneToMsg(walls));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
