#include "roi_extractor/roi_extractor.h"

ROIExtractor::ROIExtractor()
//    :_plane_clipper(Eigen::Vector4f(0,0,0,0))
{
#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.addCoordinateSystem (1.0);
    _viewer.initCameraParameters ();

    _colors.push_back(common::Color(255,0,0));
    _colors.push_back(common::Color(0,255,0));
    _colors.push_back(common::Color(0,0,255));
    _colors.push_back(common::Color(255,255,0));
    _colors.push_back(common::Color(0,255,255));
#endif
}

ROIExtractor::~ROIExtractor()
{}

void ROIExtractor::set_rgb_image(const cv_bridge::CvImageConstPtr& img)
{
    _rgb = img;
}

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
        double wall_thickness = 0.1)
{
    using namespace Eigen;

    common::vision::ROIArrayPtr rois(new std::vector<common::vision::ROI>);

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

    std::cerr << "Points remaining: " << filtered.indices.size() << std::endl;

    common::PointCloudRGB::Ptr cloud_t(new common::PointCloudRGB);
    pcl::copyPointCloud(*pcloud,filtered.indices,*cloud_t);


#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();


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

    for (int i = 0; i < clusters.size(); ++i)
    {
        common::PointCloudRGB::Ptr cluster_cloud(new common::PointCloudRGB);

        pcl::PointIndices& cluster = clusters[i];
        pcl::copyPointCloud(*cloud_t, cluster.indices, *cluster_cloud);

#if ENABLE_VISUALIZATION_ROIS==1
        std::stringstream ss;
        ss << "Cluster" << i;
        pcl::visualization::AddPointCloud(_viewer,cluster_cloud,ss.str(),_colors[i].r,_colors[i].g,_colors[i].b);
#endif

        common::vision::ROI roi(cluster_cloud);
        rois->push_back(roi);
    }

#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.spinOnce(100);
#endif

    return rois;
}
