#include "fdcl/matrix_utils.hpp"


fdcl::Matrix3 fdcl::hat(const Vector3 v)
{
    Matrix3 V;
    V.setZero();

    V(2,1) = v(0);
    V(1,2) = -V(2, 1);
    V(0,2) = v(1);
    V(2,0) = -V(0, 2);
    V(1,0) = v(2);
    V(0,1) = -V(1, 0);

    return V;
}


fdcl::Vector3 fdcl::vee(const Matrix3 V)
{
    // TODO: improve code by: https://codereview.stackexchange.com/questions/77546/multiply-vector-elements-by-a-scalar-value-using-stl-and-templates
    Vector3 v;
    Matrix3 E;

    v.setZero();
    E = V + V.transpose();
    if(E.norm() > 1.e-6)
    {
        std::cout << "vee: E.norm() = " << E.norm() << std::endl;
    }
    //	else
    //	{
    v(0) = V(2, 1);
    v(1) = V(0, 2);
    v(2) = V(1, 0);
    //	}

    return  v;
}


double fdcl::sinx_over_x(const double x)
{
    double y;
    double eps = 1.e-6;

    if(abs(x) < eps)
    {
        y = - pow(x, 10) / 39916800. + pow(x, 8) / 362880.
            - pow(x, 6) / 5040. + pow(x, 4) / 120. - pow(x, 2) / 6. + 1.;
    }
    else
    {
        y = sin(x) / x;
    }

    return y;
}


fdcl::Matrix3 fdcl::expm_SO3(const Vector3 r)
{
    Matrix3 R;
    double theta, y, y2;

    theta = r.norm();
    y = sinx_over_x(theta);
    y2 = sinx_over_x(theta / 2.);

    R.setIdentity();
    R += y * hat(r) + 1. / 2. * pow(y2, 2) * hat(r) * hat(r);

    return R;
}


fdcl::Vector3 fdcl::logm_SO3(const Matrix3 R)
{
    Vector3 r;
    Matrix3 I;
    double eps = 1.e-6;

    r.setZero();
    I.setIdentity();

    if((I - R * R.transpose()).norm() > eps || abs(R.determinant() - 1) > eps)
    {
        std::cerr << "logm_SO3: R is not a rotation matrix" << std::endl;
    }
    else
    {
        Eigen::EigenSolver<Eigen::MatrixXd> eig(R);
        double min_del_lam_1 = 1.0, cos_theta, theta;
        int i, i_min = -1;
        Vector3 v;
        Matrix3 R_new;

        for(i = 0; i < 3; i++)
        {
            if(eig.eigenvectors().col(i).imag().norm() < eps)
            {
                if(pow(eig.eigenvalues()[i].real(), 2) - 1. < min_del_lam_1 )
                {
                    min_del_lam_1 = pow(eig.eigenvalues()[i].real(), 2) - 1.0;
                    i_min = i;
                }
            }
        }
        v = eig.eigenvectors().col(i_min).real();

        cos_theta = (R.trace() - 1.0) / 2.0;

        if(cos_theta > 1.0) cos_theta = 1.0;
        else if(cos_theta < -1.0) cos_theta = - 1.0;

        theta = acos(cos_theta);
        R_new = expm_SO3(theta * v);

        if((R - R_new).norm() > (R - R_new.transpose()).norm()) v =- v;

        r = theta * v;

        return r;
    }

    return r;
}


bool fdcl::assert_SO3(Matrix3 R, const char *R_name)
{
    bool isSO3;
    double errO, errD;
    Matrix3 eye3;
    double eps = 1e-5;

    eye3.setIdentity();
    errO = (R.transpose()*R - eye3).norm();
    errD = pow(1 - R.determinant(), 2);

    if (errO > eps || errD > eps)
    {
        isSO3 = false;
        std::cerr << "Assert SO3: " << R_name << " ||I-R^TR|| = "
            << errO << ", det(R) = " << R.determinant()
            << std::endl;

        std::cout << R << std::endl << std::endl;
    }
    else
    {
        isSO3 = true;
    }

    return isSO3;
}


/* TODO: Write these as a template function, so they can accept diffrene type 
 * of data.
 */
void fdcl::saturate(Vector3 &x, const double x_min, const double x_max)
{
    for (int i = 0; i < 3; i++)
    {
        if (x(i) > x_max) x(i) = x_max;
        else if (x(i) < x_min) x(i) = x_min;
    }
}


void fdcl::saturate(Vector4 &x, const double x_min, const double x_max)
{
    for (int i = 0; i < 4; i++)
    {
        if (x(i) > x_max) x(i) = x_max;
        else if (x(i) < x_min) x(i) = x_min;
    }
}


void fdcl::saturate(Vector6 &x, const double x_min, const double x_max)
{
    for (int i = 0; i < 6; i++)
    {
        if (x(i) > x_max) x(i) = x_max;
        else if (x(i) < x_min) x(i) = x_min;
    }
}


void fdcl::saturate(int &x, const int x_min, const int x_max)
{
    if (x > x_max) 
    {
        x = x_max;
    }
    else if (x < x_min)
    {
        x = x_min;
    }
}


void fdcl::saturate(double &x, const double x_min, const double x_max)
{
    if (x > x_max)
    {
        x = x_max;
    }
    else if (x < x_min)
    {
        x = x_min;
    }
}


void fdcl::deriv_unit_vector(Vector3 B, Vector3 B_dot, \
    Vector3 &q, Vector3 &q_dot)
{
    double nB = B.norm();
    q = -B/nB;
    q_dot = hat(q)*hat(q)*B_dot/nB;
}


void fdcl::deriv_unit_vector(Vector3 B, Vector3 B_dot, Vector3 B_ddot, \
    Vector3 &q, Vector3 &q_dot, Vector3 &q_ddot)
{
    double nB = B.norm();
    q = -B/nB;
    q_dot = hat(q)*hat(q)*B_dot/nB;
    q_ddot=((2*q_dot*q.transpose()+q*q_dot.transpose())*B_dot \
        +hat(q)*hat(q)*B_ddot)/nB;
}
