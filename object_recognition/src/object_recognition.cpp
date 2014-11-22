#include <ros/ros.h>
#include <vision_msgs/ROI.h>
#include <vision_msgs/Planes.h>
#include <shape_fitting/model_fitting.h>
#include <shape_fitting/plane_fitting.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common/parameter.h>
#include <common/debug.h>
#include <common/segmented_plane.h>

#if ENABLE_VISUALIZATION_RECOGNITION==1
#include <pcl/visualization/pcl_visualizer.h>
#include <common/visualization_addons.h>
#endif

std::vector<ShapeClassifierBase*> _classifier;
pcl::ModelCoefficients::Ptr _best_coeffs;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > _clouds;
common::vision::SegmentedPlane::ArrayPtr _planes;
common::vision::SegmentedPlane* _ground_plane = NULL;

#if ENABLE_VISUALIZATION_RECOGNITION==1
pcl::visualization::PCLVisualizer _viewer("Recognition");
common::Colors _colors;
#endif

void callback_rois(const vision_msgs::ROIConstPtr& rois)
{
    while(_clouds.size() < rois->pointClouds.size()) {
        _clouds.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>));
    }

    for (int i = 0; i < rois->pointClouds.size(); ++i) {
        _clouds[i]->clear();
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
    //1.: There has to be a plane that is parallel to the ground plane
    const pcl::ModelCoefficientsConstPtr& ground_c = _ground_plane->get_coefficients();
    Eigen::Vector3f n_ground(ground_c->values[0],ground_c->values[1],ground_c->values[2]);

    int parallel_plane = 0;
    double max_metric = 0;
    for(int i = 0; i < planes.size(); ++i)
    {
        Eigen::Vector3f n_plane(planes[i].values[0],planes[i].values[1],planes[i].values[2]);

        double dot = n_ground.dot(n_plane);

        if (dot > max_metric) {
            max_metric = dot;
            parallel_plane = i;
        }
    }

    //there is no parallel plane
    if (max_metric < 0.8)
        return false;

    //--------------------------------------------------------------------------
    //2.: Distance between horizontal plane and parallel plane's centroid has to
    // be equal to the height of the cube (5cm)

    const Eigen::Vector4f& centroid = centroids[parallel_plane];
    Eigen::Vector3f parallel_centroid(centroid(0),centroid(1),centroid(2));

    if (_ground_plane->distance(parallel_centroid) < 0.04)
        return false;

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

#if ENABLE_VISUALIZATION_RECOGNITION==1
    _viewer.addCoordinateSystem (1.0);
    _viewer.initCameraParameters ();
#endif

    _classifier.push_back(new ModelFitting("Sphere",pcl::SACMODEL_SPHERE,"/vision/recognition/sphere/"));
    _classifier.push_back(new ModelFitting("Cylinder",pcl::SACMODEL_CYLINDER,"/vision/recognition/cylinder/"));
    _classifier.push_back(new PlaneFitting("Cube",2,"/vision/recognition/cube/", condition_cube));

    //Manual set of parameters
    //ros::param::set("/vision/recognition/sphere/dist_thresh",0.002);
    ros::param::set("/vision/recognition/cylinder/dist_thresh",0.01);
    ros::param::set("/vision/recognition/cylinder/normal_dist_weight",0.01);

    ros::Rate rate(10);
    common::Timer timer;
    while(n.ok()) {

        timer.start();

#if ENABLE_VISUALIZATION_RECOGNITION==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();
    _colors.reset();

#endif

        //only classify, if ground plane visible
        for(int i = 0; i < _clouds.size() && _ground_plane != NULL; ++i)
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
                ROS_ERROR("Object %d is a %s", i, _classifier[ci_max]->name().c_str());
                //std::cerr << "Object " << i << " is a " << _classifier[ci_max]->name() << std::endl;

#if ENABLE_VISUALIZATION_RECOGNITION
                //-----------------------------------------------------------------
                //Draw model
                _classifier[ci_max]->visualize(_viewer,*_best_coeffs);

                _viewer.spinOnce(1,true);
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
