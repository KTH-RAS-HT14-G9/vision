#include <roi_extractor/roi_extractor.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>
#include <wall_detector/wall_extractor.h>
#include <common/parameter.h>
#include <common/types.h>
#include <pcl_ros/point_cloud.h>
#include <pre_filter/pre_filter.h>

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

ROIExtractor _roi_extractor;
WallExtractor _wall_extractor;
PreFilter _pre_filter;

common::SharedPointCloudRGB _pcloud;
common::PointCloudRGB::Ptr _filtered;
Eigen::Matrix4f _camera_matrix;

// Parameters of wall extractor
Parameter<double> _distance_threshold("/vision/walls/dist_thresh", 0.01);
Parameter<double> _leaf_size("/vision/walls/leaf_size", 0.005);
Parameter<double> _halt_condition("/vision/walls/halt_condition", 0.2);

Parameter<double> _frustum_near("/vision/frustum/near", 0.3);
Parameter<double> _frustum_far("/vision/frustum/far", 1.7);
Parameter<double> _frustum_horz_fov("/vision/frustum/horz_fov", 60.0);
Parameter<double> _frustum_vert_fov("/vision/frustum/vert_fov", 50.0);

Parameter<int>    _outlier_meanK("/vision/walls/outliers/meanK", 50);
Parameter<double> _outlier_thresh("/vision/walls/outliers/thresh", 0.5);

// Parameters of object detector
Parameter<double> _wall_thickness("/vision/wall_filter/dist_thresh", 0.008);
Parameter<int> _cluster_min("/vision/wall_filter/cluster_min", 100);
Parameter<int> _cluster_max("/vision/wall_filter/cluster_max", 50000);
Parameter<double> _cluster_tolerance("/vision/wall_filter/cluster_tolerance", 0.01);


void callback_rgb_image(const sensor_msgs::ImageConstPtr& rgb)
{
//    cv_bridge::CvImageConstPtr cvrgb = cv_bridge::toCvShare(rgb,"");
//    _detector.set_rgb_image(cvrgb);
}

void callback_walls(const object_detector::WallsConstPtr& walls)
{

}

void callback_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pcloud)
{
    _pcloud = pcloud;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detector");

    ros::NodeHandle n;
    ros::Subscriber sub_rgb = n.subscribe<sensor_msgs::Image>("/camera/image/rgb",3,callback_rgb_image);
    ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 3, callback_point_cloud);
    ros::Publisher pub_detection = n.advertise<std_msgs::Float64MultiArray>("/vision/detector/obstacle/position",1);

    ros::Rate loop_rate(PUBLISH_FREQUENCY);
    Eigen::Vector3d leaf_size;

    _filtered = common::PointCloudRGB::Ptr(new common::PointCloudRGB);

    while(n.ok())
    {

        if (_pcloud != NULL && !_pcloud->empty())
        {
            ros::Time time = ros::Time::now();

            _filtered->clear();
            _pre_filter.set_frustum_culling(_frustum_near(), _frustum_far(), _frustum_horz_fov(), _frustum_vert_fov());
            _pre_filter.set_outlier_removal(_outlier_meanK(), _outlier_thresh());
            _pre_filter.set_voxel_leaf_size(_leaf_size(),_leaf_size(),_leaf_size());
            _pre_filter.filter(_pcloud,_filtered);

            _roi_extractor.set_cluster_constraints(_cluster_tolerance(), _cluster_min(), _cluster_max());

            leaf_size.setConstant(_leaf_size());
            common::vision::SegmentedPlane::ArrayPtr walls = _wall_extractor.extract(_filtered,_distance_threshold(),_halt_condition(),leaf_size);
            common::vision::ROIArrayPtr rois = _roi_extractor.extract(walls,_filtered,_wall_thickness());

            ROS_INFO("Time spent on vision: %lf\n", (ros::Time::now().toSec() - time.toSec()));
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
