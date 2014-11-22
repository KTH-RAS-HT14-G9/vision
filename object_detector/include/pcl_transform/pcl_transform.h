#ifndef PCL_TRANSFORM_H
#define PCL_TRANSFORM_H

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/common/geometry.h>
#include <pcl_ros/transforms.h>
#include <common/debug.h>
#include <common/types.h>
#include <common/segmented_plane.h>


class PclTransform {
public:
    PclTransform();

    void calibrate(const common::vision::SegmentedPlane& ground_plane);
    void transform(const common::SharedPointCloudRGB& in, common::PointCloudRGB::Ptr& out);

protected:

    tf::Transform _transform;

};

PclTransform::PclTransform() {
    _transform.setIdentity();
}

void PclTransform::calibrate(const common::vision::SegmentedPlane& ground_plane)
{
    Eigen::Vector3f cam_origin(0,0,0);

    double y = ground_plane.distance(cam_origin);

    pcl::ModelCoefficientsConstPtr coeff = ground_plane.get_coefficients();
    Eigen::Vector3f normal(coeff->values[0],coeff->values[1],coeff->values[2]);

    Eigen::Vector3f up(0,1,0);

    double pitch = M_PI - acos(up.dot(normal) / (normal.norm() * up.norm()));

    _transform.setOrigin(tf::Vector3(0,y,0));
    tf::Quaternion q;
    q.setRPY(0,pitch,0);
    _transform.setRotation(q);

    ROS_ERROR("Y = %.3f, Pitch = %.3f",y,RAD2DEG(pitch));
}

void PclTransform::transform(const common::SharedPointCloudRGB &in, common::PointCloudRGB::Ptr &out)
{
    //pcl_ros::transformPointCloud<pcl::PointXYZRGB>(in, out, _transform);
}


#endif // PCL_TRANSFORM_H
