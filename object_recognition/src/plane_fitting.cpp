#include <shape_fitting/plane_fitting.h>
#include <pcl/common/centroid.h>

PlaneFitting::PlaneFitting(const std::string &name,
                           int min_planes,
                           const std::string &parameter_prefix,
                           PlaneCondition* condition)
    :ShapeClassifierBase(name)
    ,_distance_threshold(parameter_prefix+"dist_thresh", 0.005)
    ,_halt_condition(parameter_prefix+"halt_condition", 0.05)
    ,_min_planes(min_planes)
    ,_condition(condition)
{
    // Optional
    _seg.setOptimizeCoefficients (true);
    // Mandatory
    _seg.setModelType (pcl::SACMODEL_PLANE);
    _seg.setMethodType(pcl::SAC_RANSAC);

    _indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
    _inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
}


void PlaneFitting::set_parameters()
{
    _seg.setDistanceThreshold(_distance_threshold());
}

double PlaneFitting::rectangular_measure(std::vector<pcl::ModelCoefficients> planes)
{
    double metric = 1.0;

    for(int i = 0; i < planes.size(); ++i) {

        pcl::ModelCoefficients& plane = planes[i];
        Eigen::Vector3d normal_i(plane.values[0],plane.values[1],plane.values[2]);
        normal_i.normalize();

        for(int k = i+1; k < planes.size(); ++k) {

            pcl::ModelCoefficients& plane2 = planes[k];
            Eigen::Vector3d normal_k(plane2.values[0],plane2.values[1],plane2.values[2]);
            normal_k.normalize();

            double dot = std::abs(normal_i.dot(normal_k));


            metric *= (1.0 - dot);
        }
    }

    return metric;
}

int PlaneFitting::rebuild_indices_for_inlier_flag(pcl::PointIndices::Ptr& indices, std::vector<bool> flags, bool condition)
{
    _indices->indices.clear();
    int n = 0;
    for(int i = 0; i < _all_inliers.size(); ++i) {
        if(_all_inliers[i]==condition) {
            _indices->indices.push_back(i);
            ++n;
        }
    }
    return n;
}

common::Classification PlaneFitting::classify(const common::SharedPointCloudRGB &cloud, pcl::ModelCoefficients::Ptr &coefficients)
{
    set_parameters();

    int N = cloud->points.size();

    _planes.clear();
    _centroids.clear();
    _indices->indices.clear();
    _inliers->indices.reserve(N);

    _all_inliers.clear();
    _all_inliers.resize(N);

    int remaining_n = rebuild_indices_for_inlier_flag(_indices,_all_inliers,false);

    _seg.setInputCloud (cloud);

    //int i = 0;

    // While halt_condition% of the original cloud is still there
    while (remaining_n > _halt_condition() * N)
    {
        pcl::ModelCoefficients coeff;
        Eigen::Vector4f centroid;

        _inliers->indices.clear();

        // Segment the largest planar component from the remaining cloud
        _seg.setIndices(_indices);
        _seg.segment (*_inliers, coeff);

        if (_inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        //mark inlier points
        for(int i = 0; i < _inliers->indices.size(); ++i)
            _all_inliers[_inliers->indices[i]] = true;

        _planes.push_back(coeff);
        pcl::compute3DCentroid(*cloud,*_inliers,centroid);
        _centroids.push_back(centroid);

        //rebuild indices
        remaining_n = rebuild_indices_for_inlier_flag(_indices,_all_inliers,false);

    }

    double probability = 0;

    if (_planes.size() > 0 && _planes.size() >= _min_planes) {

        if (_condition->condition(_planes,_centroids) == false)
            return common::Classification(name(),0);

        build_multiple_plane_model(_planes, coefficients);

        double nInliers = N - _indices->indices.size();
        double nPoints = N;

        probability = nInliers / nPoints;

        if (_planes.size() > 1)
            probability *= rectangular_measure(_planes);
    }

    return common::Classification(name(),probability);
}

void PlaneFitting::build_multiple_plane_model(std::vector<pcl::ModelCoefficients>& planes, pcl::ModelCoefficients::Ptr& coefficients)
{
    if (planes.empty()) return;

    coefficients->values.resize(planes.size()*4);

    int k = 0;
    for (int i = 0; i < planes.size(); ++i)
    {
        for(int j = 0; j < 4; ++j, ++k)
        {
            coefficients->values[k] = planes[i].values[j];
        }
    }
}

#ifdef ENABLE_VISUALIZATION_RECOGNITION
void PlaneFitting::visualize(pcl::visualization::PCLVisualizer &viewer, const pcl::ModelCoefficients &coefficients)
{

    for(int i = 0; i < coefficients.values.size(); i+=4)
    {
        pcl::ModelCoefficients plane;
        plane.values.resize(4);
        for(int k = 0; k < 4; ++k) {
            plane.values[k] = coefficients.values[i+k];
        }

        std::stringstream ss;
        ss << "Cube_" << i/4;
        viewer.addPlane(plane,0,0,0,ss.str());
    }
}
#endif
