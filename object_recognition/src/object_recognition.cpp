#include <ros/ros.h>
#include <object_detector/ROI.h>
#include <shape_fitting/shape_fitting.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common/parameter.h>
#include <common/debug.h>

#if ENABLE_VISUALIZATION_RECOGNITION==1
#include <pcl/visualization/pcl_visualizer.h>
#include <common/visualization_addons.h>
#endif


Parameter<double> _sphere_min_r("/vision/recognition/sphere/min_radius", 0.03/2.0);
Parameter<double> _sphere_max_r("/vision/recognition/sphere/max_radius", 0.07/2.0);
Parameter<int> _sphere_kd_k("/vision/recognition/sphere/kd_k", 25);
Parameter<double> _sphere_dist_thresh("/vision/recognition/sphere/dist_thresh", 0.05);
Parameter<double> _sphere_normal_weight("/vision/recognition/sphere/normal_dist_weight", 0.1);

ShapeFitting _sphere_fitter(pcl::SACMODEL_SPHERE);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > _clouds;
pcl::ModelCoefficients _model_coefficients;

#if ENABLE_VISUALIZATION_RECOGNITION==1
pcl::visualization::PCLVisualizer _viewer("Recognition");
common::Colors _colors;
#endif

void callback_rois(const object_detector::ROIConstPtr& rois)
{
    while(_clouds.size() < rois->pointClouds.size()) {
        _clouds.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>));
    }

    for (int i = 0; i < rois->pointClouds.size(); ++i) {
        _clouds[i]->clear();
        pcl::fromROSMsg<pcl::PointXYZRGB>(rois->pointClouds[i],*_clouds[i]);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_recognition");

    ros::NodeHandle n;

    ros::Subscriber sub_rois = n.subscribe<object_detector::ROI>("/vision/obstacles/rois",1,callback_rois);

#if ENABLE_VISUALIZATION_RECOGNITION==1
    _viewer.addCoordinateSystem (1.0);
    _viewer.initCameraParameters ();
#endif


    ros::Rate rate(10);
    common::Timer timer;
    while(n.ok()) {

        _sphere_fitter.set_segmentation(_sphere_dist_thresh(), _sphere_normal_weight(), _sphere_min_r(), _sphere_max_r());
        _sphere_fitter.set_normal_estimation(_sphere_kd_k());

        timer.start();

#if ENABLE_VISUALIZATION_RECOGNITION==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();
    _colors.reset();

#endif

        for(int i = 0; i < _clouds.size(); ++i)
        {
            double prob = _sphere_fitter.classify(_clouds[i],_model_coefficients);
            std::cerr << "Probability: " << prob << std::endl;

#if ENABLE_VISUALIZATION_RECOGNITION==1
            std::stringstream ss;
            ss << "Cloud_" << i;
            common::Color c = _colors.next();
            pcl::visualization::AddPointCloud(_viewer,_clouds[i],ss.str(),c.r,c.g,c.b);
            _viewer.addSphere(_model_coefficients,"sphere");

            _viewer.spinOnce(100);
#endif

        }

        ROS_INFO("Recognition - Time: %lf",timer.elapsed());

        std::cerr << "---------------------------------------------" << std::endl << std::endl;

        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
