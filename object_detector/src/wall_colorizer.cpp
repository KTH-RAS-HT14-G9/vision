#include <ros/ros.h>
#include "wall_detector/wall_extractor.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <object_detector/Walls.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Core>
#include <common/parameter.h>
#include <pre_filter/pre_filter.h>

//------------------------------------------------------------------------------
// Constants

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

WallExtractor _wall_extractor;
PreFilter _pre_filter;

common::SharedPointCloudRGB _pcloud;
common::PointCloudRGB::Ptr _filtered;
Eigen::Matrix4f _camera_matrix;

// Parameters of pre filter
Parameter<double> _frustum_near("/vision/filter/frustum/near", 0.3);
Parameter<double> _frustum_far("/vision/filter/frustum/far", 1.7);
Parameter<double> _frustum_horz_fov("/vision/filter/frustum/horz_fov", 60.0);
Parameter<double> _frustum_vert_fov("/vision/filter/frustum/vert_fov", 50.0);
Parameter<double> _leaf_size("/vision/filter/down_sampling/leaf_size", 0.003); //voxel downsampling
Parameter<bool> _fast_down_sampling("/vision/filter/down_sampling/enable_fast", false); //fast downsampling
Parameter<int> _down_sample_target_n("/vision/filter/down_sampling/fast_target_n", 5000); //fast downsampling

// Parameters of wall extractor
Parameter<double> _distance_threshold("/vision/walls/dist_thresh", 0.01);
Parameter<double> _halt_condition("/vision/walls/halt_condition", 0.2);
Parameter<double> _samples_max_dist("/vision/walls/samples_max_dist", 0.2);

Parameter<int>    _outlier_meanK("/vision/walls/outliers/meanK", 50);
Parameter<double> _outlier_thresh("/vision/walls/outliers/thresh", 0.5);

//------------------------------------------------------------------------------
// Callbacks

void callback_point_cloud(const common::SharedPointCloudRGB& pcloud)
{
    _pcloud = pcloud;
}

void callback_camera_matrix(const sensor_msgs::CameraInfoConstPtr& m)
{
    _camera_matrix <<   m->D[0], m->D[1], m->D[2], m->D[3],
                        m->K[0], m->K[1], m->K[2], m->K[3],
                        m->R[0], m->R[1], m->R[2], m->R[3],
                        m->P[0], m->P[1], m->P[2], m->P[3];

//    _wall_extractor.set_camera_matrix(_camera_matrix);
}

//------------------------------------------------------------------------------
// Test cases

common::PointCloudRGB::Ptr testcase() {

    common::PointCloudRGB::Ptr cloud(new common::PointCloudRGB);

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

    ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >
            ("/camera/depth/points", 3, callback_point_cloud);
//    ros::Subscriber sub_camera_m = n.subscribe<sensor_msgs::CameraInfo>
//            ("/camera/ir/camera_info", 1, callback_camera_matrix);

    ros::Rate rate(PUBLISH_FREQUENCY);

    _filtered = common::PointCloudRGB::Ptr(new common::PointCloudRGB);

    while(n.ok())
    {
        if (_pcloud != NULL && !_pcloud->empty())
        {
            leaf_size.setConstant(_leaf_size());

            _filtered->clear();
            _pre_filter.set_frustum_culling(_frustum_near(), _frustum_far(), _frustum_horz_fov(), _frustum_vert_fov());
            _pre_filter.set_outlier_removal(_outlier_meanK(), _outlier_thresh());
            _pre_filter.set_voxel_leaf_size(_leaf_size(),_leaf_size(),_leaf_size());
            _pre_filter.filter(_pcloud,_filtered);


            common::vision::SegmentedPlane::ArrayPtr walls = _wall_extractor.extract(_filtered,_distance_threshold(),_halt_condition(),leaf_size,_samples_max_dist());
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
