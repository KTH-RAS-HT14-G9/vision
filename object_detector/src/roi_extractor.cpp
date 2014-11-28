#include "roi_extractor/roi_extractor.h"

ROIExtractor::ROIExtractor()
//    :_plane_clipper(Eigen::Vector4f(0,0,0,0))
{
#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.addCoordinateSystem (1.0);
    _viewer.initCameraParameters ();

#endif
}

ROIExtractor::~ROIExtractor()
{}

//void ROIExtractor::set_rgb_image(const cv_bridge::CvImageConstPtr& img)
//{
//    _rgb = img;
//}

void ROIExtractor::set_cluster_constraints(double cluster_tolerance,
                                           int min_cluster_size,
                                           int max_cluster_size)
{
    _cluster_ex.setClusterTolerance(cluster_tolerance);
    _cluster_ex.setMinClusterSize(min_cluster_size);
    _cluster_ex.setMaxClusterSize(max_cluster_size);
}

bool ROIExtractor::point_is_on_plane(const pcl::PointXYZRGB& p, const pcl::ModelCoefficientsConstPtr& plane, double max_distance)
{
    double dist = (plane->values[0] * p.x + plane->values[1] * p.y + plane->values[2] * p.z + plane->values[3]);
    return dist < max_distance/2.0 && dist > -max_distance/2.0;
}

void ROIExtractor::filter_points_on_plane(const common::SharedPointCloudRGB& cloud_in,
                                          pcl::PointIndices& indices_in_out,
                                          const pcl::ModelCoefficientsConstPtr& plane,
                                          double max_distance)
{
    _index_buffer.clear();
    _index_buffer.reserve(cloud_in->size());

    int i = 0;
    for (std::vector<int>::const_iterator it = indices_in_out.indices.begin(); it != indices_in_out.indices.end(); ++it)
    {
        int idx = *it;

        if (!point_is_on_plane(cloud_in->at(idx), plane, max_distance))
            _index_buffer.push_back(idx);
    }

    std::swap(indices_in_out.indices, _index_buffer);
}

common::vision::ROIArrayPtr ROIExtractor::extract(
        const common::vision::SegmentedPlane::ArrayPtr& walls,
        const common::SharedPointCloudRGB& pcloud,
        double wall_thickness = 0.01,
        double max_obj_height = 0.05)
{
    using namespace Eigen;

    common::vision::ROIArrayPtr rois(new std::vector<common::vision::ROI>);

    //If theres is no ground plane, return
    if (walls->empty() || !walls->at(0).is_ground_plane())
        return rois;

    pcl::PointIndices filtered;
    filtered.indices = _point_indices;
    filtered.indices.clear();
    filtered.indices.reserve(pcloud->size());

    //generate indices
    for(int i = 0; i < pcloud->size(); ++i) {filtered.indices.push_back(i);}

    //remove points that lie on plane
    for (int i = 0; i < walls->size(); ++i)
    {
        pcl::ModelCoefficientsConstPtr plane = walls->at(i).get_coefficients();
        filter_points_on_plane(pcloud,filtered,plane,wall_thickness);
    }

    common::PointCloudRGB::Ptr cloud_t(new common::PointCloudRGB);
    pcl::copyPointCloud(*pcloud,filtered.indices,*cloud_t);


#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();
    _colors.reset();


//    pcl::visualization::AddPointCloud(_viewer,cloud_t,"Clipped",255,0,0);

    for (int i = 0; i < walls->size(); ++i)
    {
        std::stringstream ss;
        ss << "Plane" << i;
        const pcl::ModelCoefficientsConstPtr& plane = walls->at(i).get_coefficients();


        pcl::ModelCoefficients top = *plane;
        top.values[3] += wall_thickness/2.0;
        pcl::ModelCoefficients bottom = *plane;
        bottom.values[3] -= wall_thickness/2.0;
        _viewer.addPlane(top, 0,0,0, ss.str());
        ss << "_bottom";
        _viewer.addPlane(bottom, 0,0,0, ss.str());
    }

#endif


    //Cluster remaining cloud points
    std::vector<pcl::PointIndices> clusters;

    _cluster_ex.setInputCloud(cloud_t);
    _cluster_ex.extract(clusters);

    int ground_plane = 0;

#if ENABLE_VISUALIZATION_ROIS==1

    std::stringstream ss;
    ss << "Ground";
    const pcl::ModelCoefficientsConstPtr& plane = walls->at(ground_plane).get_coefficients();

    pcl::ModelCoefficients top = *plane;
    top.values[3] += max_obj_height;
    pcl::ModelCoefficients bottom = *plane;
    bottom.values[3] -= max_obj_height;
    _viewer.addPlane(top, 0,0,0, ss.str());
    ss << "_bottom";
    _viewer.addPlane(bottom, 0,0,0, ss.str());

    int i = 0;
#endif

    //remove cluster that are not inside max object height
    for(std::vector<pcl::PointIndices>::iterator itCluster = clusters.begin(); itCluster != clusters.end(); ++itCluster)
    {
        bool enclosed = true;

        std::vector<int>& indices = itCluster->indices;
        for(std::vector<int>::iterator itPoints = indices.begin(); itPoints != indices.end(); ++itPoints)
        {
            int idx = *itPoints;
            const pcl::PointXYZRGB& p = cloud_t->at(idx);
            if(!point_is_on_plane(p,walls->at(ground_plane).get_coefficients(),max_obj_height*2.0)) {
                enclosed = false;
                break;
            }
        }

        //add to result set if cluster is close enough to do some reasonable
        //classification
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_t,indices,centroid);
        double dist = centroid.head<2>().norm();
        //ROS_ERROR("Distance to ROI: %.3lf",dist);
        if (enclosed && dist < 0.4)
        {
            common::PointCloudRGB::Ptr cluster_cloud(new common::PointCloudRGB);

            pcl::copyPointCloud(*cloud_t, indices, *cluster_cloud);

    #if ENABLE_VISUALIZATION_ROIS==1
            std::stringstream ss;
            ss << "Cluster" << i++;
            common::Color c = _colors.next();
            pcl::visualization::AddPointCloud(_viewer,cluster_cloud,ss.str(),c.r,c.g,c.b);
    #endif

            common::vision::ROI roi(cluster_cloud);
            rois->push_back(roi);
        }
    }


#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.spinOnce(1,true);
#endif

    return rois;
}
