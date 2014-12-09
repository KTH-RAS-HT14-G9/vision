#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vision_msgs/ROI.h>
#include <vision_msgs/Planes.h>
#include <vision_msgs/Object.h>
#include <color_detection/color_detector.h>
#include <object_recognition/shape_recognition.h>
#include <object_recognition/object_confirmation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common/parameter.h>
#include <common/debug.h>
#include <common/segmented_plane.h>
#include <common/object_classification.h>
#include <common/marker_delegate.h>

#include <ras_msgs/RAS_Evidence.h>
#include <sensor_msgs/Image.h>

#ifdef ENABLE_VISUALIZATION_RECOGNITION
#include <pcl/visualization/pcl_visualizer.h>
#include <common/visualization_addons.h>
#endif

ColorDetector _classifier_color;
ShapeRecognition _classifier_shape;
ObjectConfirmation _object_confirmation;
common::MarkerDelegate _marker_delegate("robot","objects");

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > _clouds;
common::vision::SegmentedPlane::ArrayPtr _planes;
common::vision::SegmentedPlane* _ground_plane = NULL;

sensor_msgs::Image::ConstPtr _img;

std::vector<common::ObjectClassification > _classifications;

int _recognition_phase = PHASE_DETECTION;

int _num_rois = 0;

#ifdef ENABLE_VISUALIZATION_RECOGNITION
boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer(new pcl::visualization::PCLVisualizer("Recognition"));
common::Colors _colors;
#endif

void callback_image(const sensor_msgs::Image::ConstPtr& msg)
{
    _img=msg;
}

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

int classificationToTypeID(common::ObjectClassification& classification)
{
    const std::string& shape = classification.shape().name();
    const std::string& color = classification.color().name();

    if (shape.compare("Cube") == 0)
    {
        if (color.compare("red") == 0) return vision_msgs::Object::TYPE_RED_CUBE;
        if (color.compare("green") == 0) return vision_msgs::Object::TYPE_GREEN_CUBE;
        if (color.compare("blue") == 0) return vision_msgs::Object::TYPE_BLUE_CUBE;
        if (color.compare("yellow") == 0) return vision_msgs::Object::TYPE_YELLOW_CUBE;
    }
    if (shape.compare("Ball") == 0)
    {
        if (color.compare("red") == 0) return vision_msgs::Object::TYPE_RED_BALL;
        if (color.compare("yellow") == 0) return vision_msgs::Object::TYPE_YELLOW_BALL;
    }
    if (shape.compare("Cylinder") == 0)
    {
        if (color.compare("green") == 0) return vision_msgs::Object::TYPE_GREEN_CYLINDER;
    }
    if (shape.compare("Triangle") == 0)
    {
        if (color.compare("blue") == 0) return vision_msgs::Object::TYPE_BLUE_TRIANGLE;
    }
    if (shape.compare("Cross") == 0)
    {
        if (color.compare("purple") == 0) return vision_msgs::Object::TYPE_PURPLE_CROSS;
    }
    if (shape.compare("Patric") == 0)
    {
        return vision_msgs::Object::TYPE_PATRIC;
    }

    ROS_ERROR("Could not convert %s %s to object type id.",shape.c_str(),color.c_str());
    return vision_msgs::Object::TYPE_UNKNOWN;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_recognition");

    _planes = common::vision::SegmentedPlane::ArrayPtr(new std::vector<common::vision::SegmentedPlane>());

    ros::NodeHandle n;

    ros::Subscriber sub_rois = n.subscribe<vision_msgs::ROI>("/vision/obstacles/rois",3,callback_rois);
    ros::Subscriber sub_planes = n.subscribe<vision_msgs::Planes>("/vision/obstacles/planes",3,callback_planes);
    ros::Publisher pub_espeak = n.advertise<std_msgs::String>("/espeak/string",1);
    ros::Publisher pub_viz = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",10);

    ros::Subscriber sub_img = n.subscribe<sensor_msgs::Image>("camera/rgb/image_raw",3,callback_image);
    ros::Publisher pub_evidence = n.advertise<ras_msgs::RAS_Evidence>("/evidence",10);
    ros::Publisher pub_obj_msg = n.advertise<vision_msgs::Object>("/vision/obstacle/object",10);

#ifdef ENABLE_VISUALIZATION_RECOGNITION
    _viewer->addCoordinateSystem (1.0);
    _viewer->initCameraParameters ();
    _classifier_shape.set_viewer(_viewer);
#endif

    ros::Rate rate(10);
    common::Timer timer;
    while(n.ok()) {

        timer.start();

#ifdef ENABLE_VISUALIZATION_RECOGNITION
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
            common::Classification classification_shape;
            common::Classification classification_color;


#ifdef ENABLE_VISUALIZATION_RECOGNITION
            //-----------------------------------------------------------------
            //Draw cloud
            std::stringstream ss;
            ss << "Cloud_" << i;
            common::Color c = _colors.next();
            pcl::visualization::AddPointCloud(*_viewer,_clouds[i],ss.str(),c.r,c.g,c.b);

            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_est;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            normal_est.setSearchMethod(kd_tree);
            double kd_k;
            ros::param::get("/vision/recognition/cylinder/kd_k",kd_k);
            normal_est.setKSearch(kd_k);


            normal_est.setInputCloud(_clouds[i]);
            normal_est.compute(*normals);

            _viewer->addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(_clouds[i],normals,5);


            //_classifiers[ci_max]->visualize(*_viewer,*_best_coeffs);

            _viewer->spinOnce(30,true);

#endif

            //determine shape
            classification_shape = _classifier_shape.classify(_clouds[i],_planes,_ground_plane);

            //determine color
            classification_color = _classifier_color.classify(_clouds[i]);
            classification_color.set_shape_attributes(classification_shape.coefficients(), classification_shape.centroid(), classification_shape.obb());

            common::ObjectClassification classified_object(classification_shape,classification_color);
            _classifications.push_back(classified_object);
        }

        //----------------------------------------------------------------------
        // Determine strongest ROI classification

        double max_color_prob = 0;
        int max_color_i = -1;
        for (int i = 0; i < _classifications.size(); ++i)
        {
            if (_classifications[i].color().probability() > max_color_prob)
            {
                max_color_prob = _classifications[i].color().probability();
                max_color_i = i;
            }
        }


        double max_shape_prob = 0;
        int max_shape_i = -1;
        for (int i = 0; i < _classifications.size(); ++i)
        {
            if (_classifications[i].shape().probability() > max_shape_prob)
            {
                max_shape_prob = _classifications[i].shape().probability();
                max_shape_i = i;
            }
        }


        common::Classification strongest_shape;
        common::Classification strongest_color;
        if (max_shape_i >= 0) {
            strongest_shape = _classifications[max_shape_i].shape();
        }
        if (max_color_i >= 0) {
            strongest_color = _classifications[max_color_i].color();
        }

        common::ObjectClassification strongest_classification(strongest_shape,strongest_color);


        //------------------------------------------------------------------------------
        // Send classifications to object confirmation

        common::ObjectClassification classified_object;
        if(_object_confirmation.update(strongest_classification,classified_object,_recognition_phase))
        {
            //------------------------------------------------------------------------------
            // Publish messages
            if (!classified_object.shape().is_undefined() && !classified_object.color().is_undefined())
            {
                //------------------------------------------------------------------------------
                // Publish object message
                vision_msgs::Object obj_msg;
                const Eigen::Vector3f& centroid = classified_object.shape().centroid();
                obj_msg.x = centroid(0);
                obj_msg.y = centroid(1);

                //TODO: Fix me
                if (obj_msg.x == 0)
                    obj_msg.x = 0.5;

                if (_recognition_phase == PHASE_RECOGNITION)
                    obj_msg.type = classificationToTypeID(classified_object);
                else
                    obj_msg.type = vision_msgs::Object::TYPE_UNKNOWN;

                ROS_ERROR("Type: %d, Obj (%.3lf ,%.3lf)", obj_msg.type, obj_msg.x,obj_msg.y);

                pub_obj_msg.publish(obj_msg);

                //only publish evidence when we have classified in phase 2
                if (_recognition_phase == PHASE_RECOGNITION &&
                        obj_msg.type != vision_msgs::Object::TYPE_UNKNOWN)
                {
                    //------------------------------------------------------------------------------
                    // Publish evidence
                    ROS_ERROR("Publishing %s to espeak.", classified_object.espeak_text().c_str());
                    std_msgs::String msg;
                    msg.data = classified_object.espeak_text();
                    pub_espeak.publish(msg);

                    if (_img != NULL) {

                        ras_msgs::RAS_Evidence evidence;
                        evidence.stamp = ros::Time::now();
                        evidence.group_number = 9;
                        evidence.image_evidence = *_img;
                        evidence.object_id = classified_object.espeak_text();
                        pub_evidence.publish(evidence);

                    }


                    //------------------------------------------------------------------------------
                    // Publish marker
                    //_marker_delegate.add(classified_object);
                    //pub_viz.publish(_marker_delegate.get());
                }
            }

            //------------------------------------------------------------------------------
            // Switch phase
            if (_recognition_phase == PHASE_DETECTION) {
                _recognition_phase = PHASE_RECOGNITION;
            }
            else {
                _recognition_phase = PHASE_DETECTION;
            }
        }

        //ROS_INFO("Recognition - Time: %lf",timer.elapsed());


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
