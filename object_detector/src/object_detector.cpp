#include "obstacle_detector.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>
#include <wall_detector/wall_extractor.h>
#include <common/parameter.h>
#include <common/types.h>

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

ROIExtractor _roi_extractor;

WallExtractor _wall_extractor;
common::SharedPointCloud _pcloud;
Eigen::Matrix4f _camera_matrix;

// Parameters of wall extractor
Parameter<double> _distance_threshold("/vision/walls/dist_thresh", 0.01);
Parameter<double> _leaf_size("/vision/walls/leaf_size", 0.02);
Parameter<double> _halt_condition("/vision/walls/halt_condition", 0.2);

Parameter<double> _frustum_near("/vision/walls/frustum/near", 0.3);
Parameter<double> _frustum_far("/vision/walls/frustum/far", 1.7);
Parameter<double> _frustum_horz_fov("/vision/walls/frustum/horz_fov", 60.0);
Parameter<double> _frustum_vert_fov("/vision/walls/frustum/vert_fov", 50.0);

// Parameters of object detector


void callback_rgb_image(const sensor_msgs::ImageConstPtr& rgb)
{
//    cv_bridge::CvImageConstPtr cvrgb = cv_bridge::toCvShare(rgb,"");
//    _detector.set_rgb_image(cvrgb);
}

void callback_walls(const wall_detector::WallsConstPtr& walls)
{

}

void callback_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcloud)
{
    _pcloud = pcloud;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detector");

    ros::NodeHandle n;
    ros::Subscriber sub_rgb = n.subscribe<sensor_msgs::Image>("/camera/image/rgb",3,callback_rgb_image);
    //ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/points", 3, callback_point_cloud);
    ros::Publisher pub_detection = n.advertise<std_msgs::Float64MultiArray>("/vision/detector/obstacle/position",1);

    ros::Rate loop_rate(PUBLISH_FREQUENCY);
    Eigen::Vector3d leaf_size;

    while(n.ok())
    {
        _wall_extractor.set_frustum_culling(_frustum_near(),_frustum_far(),_frustum_horz_fov(),_frustum_vert_fov());
        leaf_size.setConstant(_leaf_size());
        SegmentedWall::ArrayPtr walls = _wall_extractor.extract(_pcloud,_distance_threshold(),_halt_condition(),leaf_size);

        //std::vector<ROIExtractor::anomaly_blob> obstacle = _roi_extractor.detect();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
