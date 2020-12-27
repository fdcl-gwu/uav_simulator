#ifndef FDCL_COMMON_TYPES
#define FDCL_COMMON_TYPES

#include "Eigen/Dense"

namespace fdcl
{

typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 4, 1> Vector4;
typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Matrix<double, 3, 3> Matrix3;

}  // End of namespace fdcl

#endif
