#include <ros/ros.h>
#include <object_detector/ROI.h>
#include <shape_fitting/shape_fitting.h>
#include <pcl_conversions/pcl_conversions.h>

ShapeFitting _sphere_fitter(pcl::SACMODEL_SPHERE);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > _clouds;

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

    ros::Subscriber sub_rois = n.subscribe<object_detector::ROI>("/vision/obstacle/rois",1,callback_rois);


    ros::Rate rate(10);
    while(n.ok()) {

        for(int i = 0; i < _clouds.size(); ++i)
        {
            double prob = _sphere_fitter.classify(_clouds[i]);
            std::cerr << "Probability: " << prob << std::endl;
        }

        std::cerr << "---------------------------------------------" << std::endl << std::endl;

        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
