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
    PclTransform(int min_calib_it, double origin_x, double origin_y, double yaw, double roll);

    bool calibrate(const common::vision::SegmentedPlane& ground_plane);
    void transform(const common::SharedPointCloudRGB& in, common::PointCloudRGB::Ptr& out);

    void transform(const common::SharedPointCloudRGB &in, common::PointCloudRGB::Ptr &out,
                   const tf::Matrix3x3& basis,
                   const tf::Vector3& origin,
                   const tf::Vector3& rotation);
protected:

    void makeOriginAndRotation(Eigen::Vector3f& origin,
                               Eigen::Vector3f& rotation,
                               double x, double y, double z,
                               double roll, double pitch, double yaw);

    void setRPY(double roll, double pitch, double yaw);
    void setOrigin(double x, double y, double z);

    void setPitch(double pitch);

    tf::Transform _transform;
    Eigen::Vector3f _cam_origin;
    Eigen::Vector3f _cam_rotation;

    double _origin_x, _origin_y;
    double _yaw, _roll;

    int _iterations;
    int _max_iterations;
};

PclTransform::PclTransform(int min_calib_it, double origin_x, double origin_y, double roll, double yaw)
    :_iterations(0)
    ,_max_iterations(min_calib_it)
    ,_origin_x(origin_x)
    ,_origin_y(origin_y)
    ,_roll(roll)
    ,_yaw(yaw)
{
    _transform.setIdentity();

    _cam_origin.setConstant(0);
    _cam_rotation.setConstant(0);
}

void PclTransform::makeOriginAndRotation(Eigen::Vector3f &origin, Eigen::Vector3f &rotation,
                                         double x, double y, double z,
                                         double roll, double pitch, double yaw)
{
    origin(0) = x;
    origin(1) = y;
    origin(2) = z;

    //set initial retransformation from z->forward to x->forward
    rotation(0) = M_PI_2 + pitch;
    rotation(1) = M_PI + roll;
    rotation(2) = M_PI_2 + yaw;
}

bool PclTransform::calibrate(const common::vision::SegmentedPlane& ground_plane)
{
    if (_iterations == _max_iterations) return true;

    double z = ground_plane.distance(Eigen::Vector3f(0,0,0));

    pcl::ModelCoefficientsConstPtr coeff = ground_plane.get_coefficients();
    Eigen::Vector3f normal(coeff->values[0],coeff->values[1],coeff->values[2]);

    Eigen::Vector3f up(0,1,0);

    double pitch = M_PI - acos(up.dot(normal) / (normal.norm() * up.norm()));


    ROS_INFO("Z = %.3f, Pitch = %.3f",z,RAD2DEG(pitch));


    Eigen::Vector3f origin;
    Eigen::Vector3f rotation;

    double x = _origin_x;
    double y = _origin_y;
    double roll = _roll;
    double yaw = _yaw;
    makeOriginAndRotation(origin, rotation, x,y,z, roll,-pitch,yaw);

    _cam_origin += origin;
    _cam_rotation += rotation;

    _iterations++;

    if (_iterations == _max_iterations) {

        _cam_origin /= _max_iterations;
        _cam_rotation /= _max_iterations;

        _transform.setOrigin(tf::Vector3(_cam_origin(0),_cam_origin(1),_cam_origin(2)));
        tf::Quaternion q;
        q.setRPY(_cam_rotation(0),_cam_rotation(1),_cam_rotation(2));
        _transform.setRotation(q);

        ROS_ERROR("Finished calibration with z= %.4lf, pitch= %.4lf", _cam_origin(2), RAD2DEG(_cam_rotation(0)-M_PI_2));

        return true;
    }

    return false;
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
