#include "roi_extractor/roi_extractor.h"

ROIExtractor::ROIExtractor()
    :_plane_clipper(Eigen::Vector4f(0,0,0,0))
{
#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.addCoordinateSystem (1.0);
    _viewer.initCameraParameters ();
    _viewer.setBackgroundColor(255,255,255,0);
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

common::vision::ROIArrayPtr ROIExtractor::extract(
        const common::vision::SegmentedPlane::ArrayPtr& walls,
        const common::SharedPointCloudRGB& pcloud,
        double wall_thickness = 0.1)
{
    using namespace Eigen;

    common::vision::ROIArrayPtr rois(new std::vector<common::vision::ROI>);


    pcl::PointIndices clipped;
    pcl::PointIndices temp;
    //remove points that lie on plane
    for (int i = 0; i < walls->size(); ++i)
    {
        pcl::ModelCoefficientsConstPtr plane = walls->at(i).get_coefficients();
        Eigen::Vector4f coeff(plane->values[0], plane->values[1], plane->values[2], plane->values[3]);

        //_plane_clipper.setPlaneParameters(coeff);
        //_plane_clipper.clipPointCloud3D(*pcloud,clipped.indices,temp.indices);

        std::swap(clipped.indices,temp.indices);
    }

    common::PointCloudRGB::Ptr cloud_t(new common::PointCloudRGB);
    pcl::copyPointCloud(*pcloud,clipped.indices,*cloud_t);


#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.removeAllPointClouds();
    _viewer.removeAllShapes();


    pcl::visualization::AddPointCloud(_viewer,cloud_t,"Clipped");
#endif


//    //Cluster remaining cloud points
//    std::vector<pcl::PointIndices> clusters;

//    _cluster_ex.setInputCloud(pcloud);
//    _cluster_ex.extract(clusters);

//    for (int i = 0; i < clusters.size(); ++i)
//    {
//        common::PointCloudRGB::Ptr cluster_cloud(new common::PointCloudRGB);

//        pcl::PointIndices& cluster = clusters[i];
//        pcl::copyPointCloud(*pcloud, cluster.indices, *cluster_cloud);

//        common::vision::ROI roi(cluster_cloud);
//        rois->push_back(roi);
//    }

#if ENABLE_VISUALIZATION_ROIS==1
    _viewer.spinOnce(100);
#endif

    return rois;
}
