#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vision_msgs/ROI.h>
#include <vision_msgs/Planes.h>
#include <color_detection/color_detector.h>
#include <object_recognition/shape_recognition.h>
#include <object_recognition/object_confirmation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common/parameter.h>
#include <common/debug.h>
#include <common/segmented_plane.h>

#if ENABLE_VISUALIZATION_RECOGNITION==1
#include <pcl/visualization/pcl_visualizer.h>
#include <common/visualization_addons.h>
#endif

ColorDetector _classifier_color;
ShapeRecognition _classifier_shape;
ObjectConfirmation _object_confirmation;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > _clouds;
common::vision::SegmentedPlane::ArrayPtr _planes;
common::vision::SegmentedPlane* _ground_plane = NULL;

std::vector<std::pair<common::NameAndProbability, common::NameAndProbability> > _classifications;

int _num_rois = 0;

#if ENABLE_VISUALIZATION_RECOGNITION==1
boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer(new pcl::visualization::PCLVisualizer("Recognition"));
common::Colors _colors;
#endif

void callback_rois(const vision_msgs::ROIConstPtr& rois)
{
    _num_rois = rois->pointClouds.size();

    while(_clouds.size() < _num_rois) {
        _clouds.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>));
    }

    for (int i = 0; i < _num_rois; ++i) {
        pcl::fromROSMsg<pcl::PointXYZRGB>(rois->pointClouds[i],*_clouds[i]);
    }

}

void callback_planes(const vision_msgs::PlanesConstPtr& msg)
{
    _ground_plane = NULL;
    _planes->clear();
    common::vision::msgToPlanes(msg, _planes);

    //find ground plane
    for (int i = 0; i < _planes->size(); ++i)
    {
        if (_planes->at(i).is_ground_plane())
        {
            _ground_plane = &_planes->at(i);
            break;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_recognition");

    _planes = common::vision::SegmentedPlane::ArrayPtr(new std::vector<common::vision::SegmentedPlane>());

    ros::NodeHandle n;

    ros::Subscriber sub_rois = n.subscribe<vision_msgs::ROI>("/vision/obstacles/rois",1,callback_rois);
    ros::Subscriber sub_planes = n.subscribe<vision_msgs::Planes>("/vision/obstacles/planes",1,callback_planes);
    ros::Publisher pub_espeak = n.advertise<std_msgs::String>("/espeak/string",1);

#if ENABLE_VISUALIZATION_RECOGNITION==1
    _viewer->addCoordinateSystem (1.0);
    _viewer->initCameraParameters ();
    _classifier_shape.set_viewer(_viewer);
#endif

    ros::Rate rate(10);
    common::Timer timer;
    while(n.ok()) {

        timer.start();

#if ENABLE_VISUALIZATION_RECOGNITION==1
    _viewer->removeAllPointClouds();
    _viewer->removeAllShapes();
    _colors.reset();

#endif

        _classifications.clear();

        //----------------------------------------------------------------------
        // Classify rois

        //only classify, if ground plane visible
        for(int i = 0; i < _num_rois && _ground_plane != NULL; ++i)
        {
            common::NameAndProbability classification_shape;
            common::NameAndProbability classification_color;


#if ENABLE_VISUALIZATION_RECOGNITION==1
            //-----------------------------------------------------------------
            //Draw cloud
            std::stringstream ss;
            ss << "Cloud_" << i;
            common::Color c = _colors.next();
            pcl::visualization::AddPointCloud(*_viewer,_clouds[i],ss.str(),c.r,c.g,c.b);
#endif

            //determine shape
            classification_shape = _classifier_shape.classify(_clouds[i],_planes,_ground_plane);

            //determine color
            classification_color = _classifier_color.classify(_clouds[i]);

            std::pair<common::NameAndProbability, common::NameAndProbability> classified_object;
            classified_object.first = classification_shape;
            classified_object.second = classification_color;
            _classifications.push_back(classified_object);
        }

        //----------------------------------------------------------------------
        // Determine strongest ROI classification

        double max_shape_prob = 0;
        int max_i = -1;
        for (int i = 0; i < _classifications.size(); ++i)
        {
            if (_classifications[i].first.probability() > max_shape_prob)
            {
                max_shape_prob = _classifications[i].first.probability();
                max_i = i;
            }
        }

        common::NameAndProbability classification_shape;
        common::NameAndProbability classification_color;

        if (max_i >= 0) {
            classification_shape = _classifications[max_i].first;
            classification_color = _classifications[max_i].second;
        }

        //------------------------------------------------------------------------------
        // Send classifications to object confirmation

        std::string classified_object;
        if(_object_confirmation.update(classification_shape,classification_color,classified_object))
        {
            ROS_ERROR("Publishing %s to espeak.", classified_object.c_str());
            std_msgs::String msg;
            msg.data = classified_object;
            pub_espeak.publish(msg);
        }

        ROS_INFO("Recognition - Time: %lf",timer.elapsed());


//        std::cerr << "---------------------------------------------" << std::endl << std::endl;

        //------------------------------------------------------------------------------
        // Clear inputs

        for (int i = 0; i < _clouds.size(); ++i) {
            _clouds[i]->clear();
            _num_rois = 0;
        }
        _classifications.clear();

        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
