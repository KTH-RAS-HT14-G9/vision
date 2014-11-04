#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "obstacle_detector.h"
#include <std_msgs/Float64MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <wall_detector/Walls.h>

const double PUBLISH_FREQUENCY = 10.0;

ObstacleDetector _detector;

void callback_rgb_image(const sensor_msgs::ImageConstPtr& rgb)
{
//    cv_bridge::CvImageConstPtr cvrgb = cv_bridge::toCvShare(rgb,"");
//    _detector.set_rgb_image(cvrgb);
}

void callback_walls(const wall_detector::WallsConstPtr& walls)
{

}

void callback_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>& pcloud)
{

}

int main(int argc, char **argv)
{
    /*ros::init(argc, argv, "object_detector");

    ros::NodeHandle n;
    ros::Subscriber sub_walls = n.subscribe<wall_detector::WallsConstPtr>("/vision/walls",3,callback_walls);
    ros::Subscriber sub_rgb = n.subscribe<sensor_msgs::Image>("/camera/image/rgb",3,callback_rgb_image);
    ros::Subscriber sub_pcloud = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >
            ("/camera/point_cloud", 3, callback_point_cloud);
    ros::Publisher pub_detection = n.advertise<std_msgs::Float64MultiArray>("/vision/detector/obstacle/position",1);

    ros::Rate loop_rate(PUBLISH_FREQUENCY);


    while(n.ok())
    {
        std::vector<ObstacleDetector::anomaly_blob> obstacle = _detector.detect();

        //TODO: redefine publish condition
        if (true)
        {
        }

        ros::spinOnce();
        loop_rate.sleep();
    }*/

    return 0;
}
