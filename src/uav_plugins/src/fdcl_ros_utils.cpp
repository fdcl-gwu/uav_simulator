#include "fdcl/ros_utils.hpp"
#include "fdcl/matrix_utils.hpp"


void fdcl::eigen_to_ignition(
    const Vector3 input, ignition::math::Vector3d &output
)
{
    output[0] = input(0);
    output[1] = input(1);
    output[2] = input(2);
}


void fdcl::ignition_to_eigen(
    const ignition::math::Vector3d input, Vector3 &output
)
{
    output(1) = input[0];
    output(2) = input[1];
    output(3) = input[2];
}


void fdcl::quaternion_to_R(const ignition::math::Quaterniond q, Matrix3 &R)
{
    fdcl::Matrix3 eye3 = fdcl::Matrix3::Identity();
    fdcl::Vector3 q13(q.X(), q.Y(), q.Z());
    double q4 = q.W();

    fdcl::Matrix3 hat_q = fdcl::hat(q13);
    R = eye3 + 2 * q4 * hat_q + 2 * hat_q * hat_q;
}