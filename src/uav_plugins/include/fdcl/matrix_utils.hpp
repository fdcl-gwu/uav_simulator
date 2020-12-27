/** \file matrix_utils.hpp
*  \brief Miscellaneous math functions
*
*  Miscellaneous math functions used in FDCL are defined here
*/

#ifndef FDCL_MATRIX_UTILS_HPP
#define FDCL_MATRIX_UTILS_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "fdcl/common_types.hpp"

namespace fdcl
{

/** \fn Matrix3 hat(const Vector3 v)
* Returns the hat map of a given 3x1 vector. This is the inverse of vee map.
* @param v vector which the hat map is needed to be operated on
* @return hat map of the input vector
*/
Matrix3 hat(const Vector3 v);


/** \fn Vector3 vee(const Matrix3 V)
* Returns the vee map of a given 3x3 matrix. This is the inverse of hat map.
* @param V matrix which the vee map is needed to be operated on
* @return vee map of the input matrix
*/
Vector3 vee(const Matrix3 V);


/** \fn double sinx_over_x(const double x)
 * Calculates and returns the value of sin(x)/x, dealing with the case where
 * x = 0.
 * @param  x input value
 * @return   the value of sin(x)/x
 */
double sinx_over_x(const double x);


/** \fn Matrix3 expm_SO3(const Vector3 r)
 * Calculates and returns the rotation matrix in SO(3) from rotation vector.
 * This is the inverse of logm_som3.
 * @param  r rotation vector (angle * axis)
 * @return   rotation matrix in SO(3) which corresponds to the input vector
 */
Matrix3 expm_SO3(const Vector3 r);


/** \fn Vector3 logm_SO3(const Matrix3 R)
 * Calculates and returns the rotation vector from rotation matrix in SO(3).
 * This is the inverse of expm_SO3.
 * @param  R rotation matrix in SO(3)
 * @return rotation vector (angle * axis)
 */
Vector3 logm_SO3(const Matrix3 R);


/** \fn bool assert_SO3(Matrix3 R,const char *R_name)
 * Check and returns if a given matrix is in SO(3)
 * @param  R      rotation matrix
 * @param  R_name name of the rotation matrix
 * @return        true if the input matrix is true, false otherwise
 */
bool assert_SO3(Matrix3 R,const char *R_name);

/** \fn void saturate(Vector3 &x, const double x_min, const double x_max)
 * Saturate the elements of a given 3x1 vector between a minimum and a maximum
 * velue.
 * @param x     vector which the elements needed to be saturated
 * @param x_min minimum value for each element
 * @param x_max maximum value for each element
 */
void saturate(Vector3 &x, const double x_min, const double x_max);


/** \fn void saturate(Vector4 &x, const double x_min, const double x_max)
 * Saturate the elements of a given 4x1 vector between a minimum and a maximum
 * velue.
 * @param x     vector which the elements needed to be saturated
 * @param x_min minimum value for each element
 * @param x_max maximum value for each element
 */
void saturate(Vector4 &x, const double x_min, const double x_max);


/** \fn void saturate(Vector6 &x, const double x_min, const double x_max)
 * Saturate the elements of a given 6x1 vector between a minimum and a maximum
 * velue.
 * @param x     vector which the elements needed to be saturated
 * @param x_min minimum value for each element
 * @param x_max maximum value for each element
 */
void saturate(Vector6 &x, const double x_min, const double x_max);


/** \fn void saturate(int &x, const int x_min, const int x_max)
 * Saturate the elements of a given integer between a minimum and a maximum
 * velue.
 * @param x     integer which needed to be saturated
 * @param x_min minimum value for x
 * @param x_max maximum value for x
 */
void saturate(int &x, const int x_min, const int x_max);


/** \fn void saturate(double &x, const double x_min, const double x_max)
 * Saturate the elements of a given integer between a minimum and a maximum
 * velue.
 * @param x     double which needed to be saturated
 * @param x_min minimum value for x
 * @param x_max maximum value for x
 */
void saturate(double &x, const double x_min, const double x_max);


/** \fn void deriv_unit_vector(Vector3 B, Vector3 B_dot, Vector3 &q, 
 * Vector3 &q_dot)
 * finds derivative q = -B/norm(B)
 */
void deriv_unit_vector(Vector3 B, Vector3 B_dot, Vector3 &q, Vector3 &q_dot);


/** \fn void deriv_unit_vector(Vector3 B, Vector3 B_dot, Vector3 B_ddot, 
 * Vector3 &q, Vector3 &q_dot, Vector3 &q_ddot)
 * finds first and second derivatives of q = -B/norm(B)
 */
void deriv_unit_vector(Vector3 B, Vector3 B_dot, Vector3 B_ddot, Vector3 &q, \
    Vector3 &q_dot, Vector3 &q_ddot);

}  // end of namespace fdcl
#endif
