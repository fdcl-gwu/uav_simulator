#ifndef FDCL_ROS_UTILS_HPP
#define FDCL_ROS_UTILS_HPP


#include "fdcl/common_types.hpp"


#include "Eigen/Dense"
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace fdcl
{

void eigen_to_ignition(
    const Vector3 input, ignition::math::Vector3d &output
);

void ignition_to_eigen(
    const ignition::math::Vector3d input, Vector3 &output
);

void quaternion_to_R(const ignition::math::Quaterniond q, Matrix3 &R);

}  // end of namespace fdcl

#endif
