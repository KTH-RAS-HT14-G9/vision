#include <object_recognition/shape_recognition.h>
#include <common/debug.h>

ShapeRecognition::ShapeRecognition()
    :_dotprod_thresh("/vision/recognition/cube/dotprod_thresh",0.8)
    ,_shape_thresh("/vision/recognition/shape_threshold",0.7)
    ,_parameter_initiated(false)
{
    _classifiers.push_back(new ModelFitting("Sphere",pcl::SACMODEL_SPHERE,"/vision/recognition/sphere/"));
    _classifiers.push_back(new ModelFitting("Cylinder",pcl::SACMODEL_CYLINDER,"/vision/recognition/cylinder/"));
    _classifiers.push_back(new PlaneFitting("Cube",2,"/vision/recognition/cube/", this));
}

ShapeRecognition::~ShapeRecognition()
{
    for(int i = 0; i < _classifiers.size(); ++i)
        delete _classifiers[i];
}

void ShapeRecognition::set_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
{
    _viewer = viewer;
}

bool ShapeRecognition::condition(const std::vector<pcl::ModelCoefficients>& planes, const std::vector<Eigen::Vector4f>& centroids)
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

common::NameAndProbability ShapeRecognition::classify(const common::PointCloudRGB::Ptr &roi,
                                                      const common::vision::SegmentedPlane::ArrayPtr &planes,
                                                      const common::vision::SegmentedPlane *ground_plane)
{
    _ground_plane = ground_plane;

    if (!_parameter_initiated) {
        _parameter_initiated = true;
        //Manual set of parameters
        ros::param::set("/vision/recognition/sphere/dist_thresh",0.001);
        ros::param::set("/vision/recognition/cylinder/dist_thresh",0.01);
        ros::param::set("/vision/recognition/cylinder/normal_dist_weight",0.01);
        ros::param::set("/vision/recognition/sphere/normal_dist_weight",0.1);
    }

    double max_probability = 0.0;

    common::NameAndProbability classification_shape;

    int ci_max = 0;
    for(int ci = 0; ci < _classifiers.size(); ++ci)
    {
        pcl::ModelCoefficients::Ptr model_coefficients(new pcl::ModelCoefficients);

        common::NameAndProbability classification = _classifiers[ci]->classify(roi,model_coefficients);
//        std::cerr << "Probability for " << classification.name() << ": " << classification.probability() << std::endl;

        if (classification.probability() > max_probability) {
            max_probability = classification.probability();
            ci_max = ci;

            _best_coeffs = model_coefficients;
            classification_shape = classification;
        }
    }

    if (max_probability > _shape_thresh()) {

#if ENABLE_VISUALIZATION_RECOGNITION == 1
        //-----------------------------------------------------------------
        //Draw model
        _classifier[ci_max]->visualize(*_viewer,*_best_coeffs);

        _viewer.spinOnce(30,true);
#endif
    }
    else {
        classification_shape = common::NameAndProbability();
        std::cerr << "No object recognized" << std::endl;
    }

    return classification_shape;
}
