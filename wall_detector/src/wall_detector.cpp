#include <ros/ros.h>
#include "wall_extractor.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <wall_detector/Walls.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Core>

//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

WallExtractor _extractor;
WallExtractor::SharedPointCloud _pcloud;
Eigen::Matrix4f _camera_matrix;

double _distance_threshold = 0.01; std::string _distance_threshold_key = "/vision/walls/distThresh";
double _leaf_size = 0.02; std::string _leaf_size_key = "/vision/walls/leafSize";
double _halt_condition = 0.2; std::string _halt_condition_key = "/vision/walls/haltCondition";

double _frustum_near = 0.3; std::string _frustum_near_key = "vision/walls/frustum/near";
double _frustum_far  = 1.7; std::string _frustum_far_key  = "vision/walls/frustum/far";
double _frustum_horz_fov = 60.0; std::string _frustum_horz_fov_key = "vision/walls/frustum/horzFOV";
double _frustum_vert_fov = 50.0; std::string _frustum_vert_fov_key = "vision/walls/frustum/vertFOV";

//------------------------------------------------------------------------------
// Callbacks

void callback_point_cloud(const WallExtractor::SharedPointCloud& pcloud)
{
    _pcloud = pcloud;
}

void callback_camera_matrix(const sensor_msgs::CameraInfoConstPtr& m)
{
    _camera_matrix <<   m->D[0], m->D[1], m->D[2], m->D[3],
                        m->K[0], m->K[1], m->K[2], m->K[3],
                        m->R[0], m->R[1], m->R[2], m->R[3],
                        m->P[0], m->P[1], m->P[2], m->P[3];

    _extractor.set_camera_matrix(_camera_matrix);
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
    //frustum:
    n.setParam(_frustum_near_key, _frustum_near);
    n.setParam(_frustum_far_key, _frustum_far);
    n.setParam(_frustum_horz_fov_key, _frustum_horz_fov);
    n.setParam(_frustum_vert_fov_key, _frustum_vert_fov);

    Eigen::Vector3d leaf_size(_leaf_size,_leaf_size,_leaf_size);

    ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >
            ("/camera/depth/points", 3, callback_point_cloud);
//    ros::Subscriber sub_camera_m = n.subscribe<sensor_msgs::CameraInfo>
//            ("/camera/ir/camera_info", 1, callback_camera_matrix);

    ros::Publisher pub_walls = n.advertise<wall_detector::Walls>("/vision/walls",10);

    ros::Rate rate(PUBLISH_FREQUENCY);

    while(n.ok())
    {
        n.getParamCached(_distance_threshold_key, _distance_threshold);
        n.getParamCached(_leaf_size_key, _leaf_size);
        n.getParamCached(_halt_condition_key, _halt_condition);
        n.getParamCached(_frustum_near_key, _frustum_near);
        n.getParamCached(_frustum_far_key, _frustum_far);
        n.getParamCached(_frustum_horz_fov_key, _frustum_horz_fov);
        n.getParamCached(_frustum_vert_fov_key, _frustum_vert_fov);

        leaf_size.setConstant(_leaf_size);
        _extractor.set_frustum_culling(_frustum_near, _frustum_far, _frustum_horz_fov, _frustum_vert_fov);

        WallExtractor::WallsPtr walls = _extractor.extract(_pcloud, _distance_threshold, _halt_condition, leaf_size);

        pub_walls.publish(generate_walls_msg(walls));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
