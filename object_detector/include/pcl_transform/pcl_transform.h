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
    PclTransform(double origin_x, double origin_y, double yaw, double roll);

    void calibrate(const common::vision::SegmentedPlane& ground_plane);
    void transform(const common::SharedPointCloudRGB& in, common::PointCloudRGB::Ptr& out);

    void transform(const common::SharedPointCloudRGB &in, common::PointCloudRGB::Ptr &out,
                   const tf::Matrix3x3& basis,
                   const tf::Vector3& origin,
                   const tf::Vector3& rotation);
protected:

    void setRPY(double roll, double pitch, double yaw);
    void setOrigin(double x, double y, double z);

    void setPitch(double pitch);

    tf::Transform _transform;
    Eigen::Vector3f _cam_origin;
    Eigen::Vector3f _cam_rotation;
};

PclTransform::PclTransform(double origin_x, double origin_y, double roll, double yaw) {
    _transform.setIdentity();

    setOrigin(origin_x,origin_y,0);
    setRPY(roll,0,yaw);
}

void PclTransform::setRPY(double roll, double pitch, double yaw)
{
    //set initial retransformation from z->forward to x->forward
    /*roll*/_cam_rotation(0) = M_PI_2 + pitch;
    /*pitch*/_cam_rotation(1) = M_PI + roll;
    /*yaw*/_cam_rotation(2) = M_PI_2 + yaw;
}

void PclTransform::setPitch(double pitch)
{
    _cam_rotation(0) = M_PI_2 + pitch;
}

void PclTransform::setOrigin(double x, double y, double z)
{
    _cam_origin(0) = x;
    _cam_origin(1) = y;
    _cam_origin(2) = z;
}

void PclTransform::calibrate(const common::vision::SegmentedPlane& ground_plane)
{
    double z = ground_plane.distance(_cam_origin);
    _cam_origin(2) = z;

    pcl::ModelCoefficientsConstPtr coeff = ground_plane.get_coefficients();
    Eigen::Vector3f normal(coeff->values[0],coeff->values[1],coeff->values[2]);

    Eigen::Vector3f up(0,1,0);

    double pitch = M_PI - acos(up.dot(normal) / (normal.norm() * up.norm()));
    setPitch(-pitch);


    _transform.setOrigin(tf::Vector3(_cam_origin(0),_cam_origin(1),_cam_origin(2)));
    tf::Quaternion q;
    q.setRPY(_cam_rotation(0),_cam_rotation(1),_cam_rotation(2));
    _transform.setRotation(q);

    ROS_INFO("Z = %.3f, Pitch = %.3f",z,RAD2DEG(pitch));
}

void PclTransform::transform(const common::SharedPointCloudRGB &in, common::PointCloudRGB::Ptr &out)
{
    pcl_ros::transformPointCloud(*in, *out, _transform);
}

void PclTransform::transform(const common::SharedPointCloudRGB &in, common::PointCloudRGB::Ptr &out,
               const tf::Matrix3x3& basis,
               const tf::Vector3& origin,
               const tf::Vector3& rotation)
{
    tf::Transform trans;
    trans.setBasis(basis);
    trans.setOrigin(origin);
    tf::Quaternion q;
    q.setRPY(rotation.x(),rotation.y(),rotation.z());
    trans.setRotation(q);

    pcl_ros::transformPointCloud(*in, *out, trans);
}


#endif // PCL_TRANSFORM_H
