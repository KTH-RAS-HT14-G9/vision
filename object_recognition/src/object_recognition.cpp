#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vision_msgs/ROI.h>
#include <vision_msgs/Planes.h>
#include <shape_fitting/model_fitting.h>
#include <shape_fitting/plane_fitting.h>
#include <color_detection/color_detector.h>
#include <object_recognition/object_confirmation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common/parameter.h>
#include <common/debug.h>
#include <common/segmented_plane.h>

#if ENABLE_VISUALIZATION_RECOGNITION==1
#include <pcl/visualization/pcl_visualizer.h>
#include <common/visualization_addons.h>
#endif

Parameter<double> _dotprod_thresh("/vision/recognition/cube/dotprod_thresh",0.8);
Parameter<double> _shape_thresh("/vision/recognition/shape_threshold",0.7);

std::vector<ShapeClassifierBase*> _classifier_shape;
ColorDetector _classifier_color;
ObjectConfirmation _object_confirmation;

pcl::ModelCoefficients::Ptr _best_coeffs;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > _clouds;
common::vision::SegmentedPlane::ArrayPtr _planes;
common::vision::SegmentedPlane* _ground_plane = NULL;

std::vector<std::pair<common::NameAndProbability, common::NameAndProbability> > _classifications;

int _num_rois = 0;

#if ENABLE_VISUALIZATION_RECOGNITION==1
pcl::visualization::PCLVisualizer _viewer("Recognition");
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

bool condition_cube(const std::vector<pcl::ModelCoefficients>& planes, const std::vector<Eigen::Vector4f>& centroids)
{

    //--------------------------------------------------------------------------
    //0.: If there are 3 planes, then it is a cube
    if (planes.size()==3) return true;

    //--------------------------------------------------------------------------
    //1.: There has to be a plane that is parallel to the ground plane
    const pcl::ModelCoefficientsConstPtr& ground_c = _ground_plane->get_coefficients();
    Eigen::Vector3f n_ground(ground_c->values[0],ground_c->values[1],ground_c->values[2]);

    int parallel_plane = 0;
    double max_metric = 0;
    for(int i = 0; i < planes.size(); ++i)
    {
        Eigen::Vector3f n_plane(planes[i].values[0],planes[i].values[1],planes[i].values[2]);

        double dot = std::abs(n_ground.dot(n_plane));

        if (dot > max_metric) {
            max_metric = dot;
            parallel_plane = i;
        }
    }

    //there is no parallel plane
    if (max_metric > _dotprod_thresh()){
        return true;
    }

    //--------------------------------------------------------------------------
    //2.: Distance between horizontal plane and parallel plane's centroid has to
    // be equal to the height of the cube (5cm)

    const Eigen::Vector4f& centroid = centroids[parallel_plane];
    Eigen::Vector3f parallel_centroid(centroid(0),centroid(1),centroid(2));

    double dist = _ground_plane->distance(parallel_centroid);
    if (dist < 0.03 || dist > 0.05){
        return false;
    }

    //--------------------------------------------------------------------------
    //3.: Centroid of the parallel plane has to have a greater distance
    // to the camera, than all the other centroids
    double parallel_dist = parallel_centroid.squaredNorm();

    for (int i = 0; i < centroids.size(); ++i)
    {
        if (i == parallel_plane)
            continue;

        const Eigen::Vector4f& c = centroids[i];
        Eigen::Vector3f plane_centroid(c(0),c(1),c(2));

        //there is a plane behind the center of the parallel plane
        if (plane_centroid.squaredNorm() > parallel_dist)
            return false;
    }


    return true;
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
    _viewer.addCoordinateSystem (1.0);
    _viewer.initCameraParameters ();
#endif

    _classifier_shape.push_back(new ModelFitting("Sphere",pcl::SACMODEL_SPHERE,"/vision/recognition/sphere/"));
    _classifier_shape.push_back(new ModelFitting("Cylinder",pcl::SACMODEL_CYLINDER,"/vision/recognition/cylinder/"));
    _classifier_shape.push_back(new PlaneFitting("Cube",2,"/vision/recognition/cube/", condition_cube));

    //Manual set of parameters
    ros::param::set("/vision/recognition/sphere/dist_thresh",0.001);
    ros::param::set("/vision/recognition/cylinder/dist_thresh",0.01);
    ros::param::set("/vision/recognition/cylinder/normal_dist_weight",0.01);
    ros::param::set("/vision/recognition/sphere/normal_dist_weight",0.1);

    ros::Rate rate(10);
    common::Timer timer;
    while(n.ok()) {

        timer.start();

#if ENABLE_VISUALIZATION_RECOGNITION==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();
    _colors.reset();

#endif

        _classifications.clear();

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
            pcl::visualization::AddPointCloud(_viewer,_clouds[i],ss.str(),c.r,c.g,c.b);
#endif

            double max_probability = 0.0;

            for(int ci = 0; ci < _classifier_shape.size(); ++ci)
            {
                pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);

                common::NameAndProbability classification = _classifier_shape[ci]->classify(_clouds[i],model_coefficients);
//                std::cerr << "Probability for " << classification.name() << ": " << classification.probability() << std::endl;

                if (classification.probability() > max_probability) {
                    max_probability = classification.probability();

                    _best_coeffs = model_coefficients;
                    classification_shape = classification;
                }
            }

            if (max_probability > _shape_thresh()) {

                //ROS_ERROR("Object %d is a %s %s", i, classification_color.name().c_str(), classification_shape.name().c_str());

#if ENABLE_VISUALIZATION_RECOGNITION
                //-----------------------------------------------------------------
                //Draw model
                _classifier[ci_max]->visualize(_viewer,*_best_coeffs);

                _viewer.spinOnce(30,true);
#endif
            }
            else {
                classification_shape = common::NameAndProbability();
                //std::cerr << "No object recognized" << std::endl;
            }

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

    for(int i = 0; i < _classifier_shape.size(); ++i)
        delete _classifier_shape[i];



    return 0;
}
