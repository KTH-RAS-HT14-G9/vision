#ifndef PRE_FILTER_H
#define PRE_FILTER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <common/debug.h>
#include <common/types.h>
#include <common/segmented_plane.h>


class PreFilter {
public:
    PreFilter();
	void set_frustum_culling(float near_plane_dist,
                             float far_plane_dist,
                             float horizontal_fov,
                             float vertical_fov)
    {
        _frustum.setNearPlaneDistance(near_plane_dist);
        _frustum.setFarPlaneDistance(far_plane_dist);
        _frustum.setHorizontalFOV(horizontal_fov);
        _frustum.setVerticalFOV(vertical_fov);
    }

    void set_outlier_removal(int meanK, double stddev_multhresh)
    {
        _sor.setMeanK(meanK);
        _sor.setStddevMulThresh(stddev_multhresh);
    }

    void set_voxel_leaf_size(double x, double y, double z)
    {
        _voxel_leaf_size(0) = x;
        _voxel_leaf_size(1) = y;
        _voxel_leaf_size(2) = z;
    }

    void filter(const common::SharedPointCloudRGB& cloud_in, common::PointCloudRGB::Ptr& cloud_out)
    {
        _indexbufA->indices.clear();
        _indexbufA->indices.reserve(cloud_in->size());

//        _indexbufB->indices.clear();
//        _indexbufB->indices.reserve(cloud_in->size());

//        _cloud_buf->clear();

        common::Timer timer;
        timer.start();

        _frustum.setInputCloud(cloud_in);
        _frustum.filter(_indexbufA->indices);

        double t_frustum = timer.elapsed();
        timer.start();

        _downsampler.setInputCloud (cloud_in);
        _downsampler.setIndices(_indexbufA);
        _downsampler.setLeafSize (_voxel_leaf_size(0), _voxel_leaf_size(1), _voxel_leaf_size(2));
        _downsampler.filter(*cloud_out);

        double t_downsample = timer.elapsed();
        timer.start();

//        _sor.setInputCloud(_cloud_buf);
//        _sor.filter(*cloud_out);

        double t_sor = timer.elapsed();

#if ENABLE_TIME_PROFILING == 1
        std::cout << "Frustum: " << t_frustum << " | Downsampler: " << t_downsample << " | SOR: " << t_sor << std::endl;
#endif
    }

protected:
    Eigen::Vector3f _voxel_leaf_size;

    pcl::FrustumCulling<pcl::PointXYZRGB> _frustum;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> _sor;
    pcl::VoxelGrid<pcl::PointXYZRGB> _downsampler;

    common::PointCloudRGB::Ptr _cloud_buf;
    pcl::PointIndices::Ptr _indexbufA;
    pcl::PointIndices::Ptr _indexbufB;
};

PreFilter::PreFilter() {
    //set_camera_matrix()
    Eigen::Matrix4f cam2robot;
    cam2robot << 0, 0, 1, 0,
                 0,-1, 0, 0,
                 1, 0, 0, 0,
                 0, 0, 0, 1;
    _frustum.setCameraPose(cam2robot);

    _indexbufA = pcl::PointIndices::Ptr(new pcl::PointIndices);
    _indexbufB = pcl::PointIndices::Ptr(new pcl::PointIndices);
    _cloud_buf = common::PointCloudRGB::Ptr(new common::PointCloudRGB);
}


#endif // PRE_FILTER_H
