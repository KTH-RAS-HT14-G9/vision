#include <roi_extractor/roi_extractor.h>
#include <wall_detector/wall_extractor.h>
#include <pre_filter/pre_filter.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_transform/pcl_transform.h>

#include <pcl_msgs/Vertices.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <vision_msgs/ROI.h>

#include <common/parameter.h>
#include <common/types.h>
#include <common/marker_delegate.h>

const double PUBLISH_FREQUENCY = 10.0;

//------------------------------------------------------------------------------
// Member variables

bool _calibrated = false;

ROIExtractor _roi_extractor;
WallExtractor _wall_extractor;
PreFilter _pre_filter;
PclTransform _pcl_transform(10,0,0,0,0);
common::MarkerDelegate _marker_delegate("robot","walls");

common::SharedPointCloudRGB _pcloud;
common::PointCloudRGB::Ptr _filtered, _transformed;
Eigen::Matrix4f _camera_matrix;

// Parameters of pre filter
Parameter<double> _frustum_near("/vision/filter/frustum/near", 0.3);
Parameter<double> _frustum_far("/vision/filter/frustum/far", 1.7);
Parameter<double> _frustum_horz_fov("/vision/filter/frustum/horz_fov", 60.0);
Parameter<double> _frustum_vert_fov("/vision/filter/frustum/vert_fov", 50.0);
Parameter<double> _leaf_size("/vision/filter/down_sampling/leaf_size", 0.003); //voxel downsampling

// Parameters of wall extractor
Parameter<double> _distance_threshold("/vision/walls/dist_thresh", 0.01);
Parameter<double> _halt_condition("/vision/walls/halt_condition", 0.2);
Parameter<double> _samples_max_dist("/vision/walls/samples_max_dist", 0.2);
Parameter<int> _down_sample_target_n("/vision/walls/down_sampling/target_n",1500);

Parameter<int>    _outlier_meanK("/vision/walls/outliers/meanK", 50);
Parameter<double> _outlier_thresh("/vision/walls/outliers/thresh", 0.5);

// Parameters of object detector
Parameter<double> _wall_thickness("/vision/rois/dist_thresh", 0.008);
Parameter<int> _cluster_min("/vision/rois/cluster_min", 100);
Parameter<int> _cluster_max("/vision/rois/cluster_max", 5000);
Parameter<double> _cluster_tolerance("/vision/rois/cluster_tolerance", 0.015);
Parameter<double> _max_object_height("/vision/rois/max_object_height", 0.07);

//------------------------------------------------------------------------------
// For Transformation
//Parameter<double> _origin_x("/transform/origin/x",0);
//Parameter<double> _origin_y("/transform/origin/y",0);
//Parameter<double> _origin_z("/transform/origin/z",0);

//Parameter<double> _rot_x("/transform/rot/roll",0);
//Parameter<double> _rot_y("/transform/rot/pitch",0);
//Parameter<double> _rot_z("/transform/rot/yaw",0);


void callback_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pcloud)
{
    _pcloud = pcloud;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detector");

    ros::NodeHandle n;
    ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 3, callback_point_cloud);
    ros::Publisher pub_detection = n.advertise<std_msgs::Float64>("/vision/detector/obstacle/distance",10);
    ros::Publisher pub_rois = n.advertise<vision_msgs::ROI>("/vision/obstacles/rois",10);
    ros::Publisher pub_planes = n.advertise<vision_msgs::Planes>("/vision/obstacles/planes",10);
    ros::Publisher pub_viz = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);

    ros::Rate loop_rate(PUBLISH_FREQUENCY);
    Eigen::Vector3d leaf_size;

    _filtered = common::PointCloudRGB::Ptr(new common::PointCloudRGB);
    _transformed = common::PointCloudRGB::Ptr(new common::PointCloudRGB);

    vision_msgs::ROIPtr roimsg = vision_msgs::ROIPtr(new vision_msgs::ROI);
    vision_msgs::PlanesPtr planesmsg = vision_msgs::PlanesPtr(new vision_msgs::Planes);

    while(n.ok())
    {

        if (_pcloud != NULL && !_pcloud->empty())
        {
            double t_prefilter = MEASURE_TIME(
                _filtered->clear();
                _pre_filter.set_frustum_culling(_frustum_near(), _frustum_far(), _frustum_horz_fov(), _frustum_vert_fov());
                _pre_filter.set_outlier_removal(_outlier_meanK(), _outlier_thresh());
                _pre_filter.set_voxel_leaf_size(_leaf_size(),_leaf_size(),_leaf_size());
                _pre_filter.filter(_pcloud,_filtered);
            );


            double t_transform = MEASURE_TIME(
                if (_calibrated) {
                    _transformed->clear();
                    _pcl_transform.transform(_filtered, _transformed);

    //                tf::Vector3 origin(_origin_x(),_origin_y(),_origin_z());
    //                tf::Vector3 rot(_rot_x(), _rot_y(), _rot_z());
    //                _pcl_transform.transform(_filtered, _transformed,basis,origin,rot);
                }
                else {
                    std::swap(*_filtered,*_transformed);
                }
            );


            common::vision::SegmentedPlane::ArrayPtr walls;
            double t_walls = MEASURE_TIME(
                _roi_extractor.set_cluster_constraints(_cluster_tolerance(), _cluster_min(), _cluster_max());

                leaf_size.setConstant(_leaf_size());
                walls = _wall_extractor.extract(_transformed,_distance_threshold(),_halt_condition(),_down_sample_target_n(),_samples_max_dist());
            );

            //calibrate
            if (_calibrated == false && walls->size() > 0 && walls->at(0).is_ground_plane()) {
                _calibrated = _pcl_transform.calibrate(walls->at(0));

                if(_calibrated==true)
                    _wall_extractor.set_down_vector(Eigen::Vector3f(0,0,-1));
            }

            common::vision::ROIArrayPtr rois;
            double t_rois = MEASURE_TIME(
            rois = _roi_extractor.extract(walls,_transformed,_wall_thickness(),_max_object_height());
            );

            //TODO: ------------------------------------------------------------
            //find closest object and publish distance to closest point

            double t_sum = t_transform+t_prefilter+t_walls+t_rois;

#if ENABLE_TIME_PROFILING == 1
            ROS_INFO("Time spent on vision. Sum: %.3lf | Transform: %.3lf | Prefilter: %.3lf | Walls: %.3lf | Rois: %.3lf\n", t_sum, t_transform, t_prefilter, t_walls, t_rois);
#endif

            roimsg->pointClouds.clear();
            common::vision::roiToMsg(rois,roimsg);
            pub_rois.publish(roimsg);

            planesmsg->planes.clear();
            common::vision::planesToMsg(walls,planesmsg);
            pub_planes.publish(planesmsg);


            if (_calibrated) {
                _marker_delegate.add(planesmsg);
                pub_viz.publish(_marker_delegate.get());
                _marker_delegate.clear();
            }

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
