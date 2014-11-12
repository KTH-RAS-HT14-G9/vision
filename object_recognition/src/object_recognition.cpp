#include <ros/ros.h>
#include <object_detector/ROI.h>
#include <shape_fitting/model_fitting.h>
#include <shape_fitting/plane_fitting.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common/parameter.h>
#include <common/debug.h>

#if ENABLE_VISUALIZATION_RECOGNITION==1
#include <pcl/visualization/pcl_visualizer.h>
#include <common/visualization_addons.h>
#endif

std::vector<ShapeClassifierBase*> _classifier;
pcl::ModelCoefficients::Ptr _best_coeffs;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > _clouds;

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

    _classifier.push_back(new ModelFitting("Sphere",pcl::SACMODEL_SPHERE,"/vision/recognition/sphere/"));
    _classifier.push_back(new ModelFitting("Cylinder",pcl::SACMODEL_CYLINDER,"/vision/recognition/cylinder/"));
    _classifier.push_back(new PlaneFitting("Cube",2,"/vision/recognition/cube/"));

    //Manual set of parameters
    ros::param::set("/vision/recognition/sphere/dist_thresh",0.002);

    ros::Rate rate(10);
    common::Timer timer;
    while(n.ok()) {

        timer.start();

#if ENABLE_VISUALIZATION_RECOGNITION==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();
    _colors.reset();

#endif

        for(int i = 0; i < _clouds.size(); ++i)
        {

#if ENABLE_VISUALIZATION_RECOGNITION==1
            //-----------------------------------------------------------------
            //Draw cloud
            std::stringstream ss;
            ss << "Cloud_" << i;
            common::Color c = _colors.next();
            pcl::visualization::AddPointCloud(_viewer,_clouds[i],ss.str(),c.r,c.g,c.b);
#endif


            double max_probability = 0.0;
            int ci_max = 0;

            for(int ci = 0; ci < _classifier.size(); ++ci)
            {
                pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);

                double prob = _classifier[ci]->classify(_clouds[i],model_coefficients);
                std::cerr << "Probability for " << _classifier[ci]->name() << ": " << prob << std::endl;

                if (prob > max_probability) {
                    max_probability = prob;
                    ci_max = ci;
                    _best_coeffs = model_coefficients;
                }
            }

            if (max_probability > 0.7) {
                std::cerr << "It's a " << _classifier[ci_max]->name() << std::endl;

#if ENABLE_VISUALIZATION_RECOGNITION
                //-----------------------------------------------------------------
                //Draw model
                _classifier[ci_max]->visualize(_viewer,*_best_coeffs);

                _viewer.spinOnce(100);
#endif
            }
            else {
                std::cerr << "No object recognized" << std::endl;
            }

        }

        ROS_INFO("Recognition - Time: %lf",timer.elapsed());

        std::cerr << "---------------------------------------------" << std::endl << std::endl;

        ros::spinOnce();
        rate.sleep();
    }

    for(int i = 0; i < _classifier.size(); ++i)
        delete _classifier[i];



    return 0;
}
