import numpy as np
# import pdb


def hat(x):
    """Returns the hat map of a given 3x1 vector.

    Args:
        x: (3x1 numpy array) vector

    Returns:
        x_hat: (3x3 numpy array) hat of the input vector
    """
    
    ensure_vector(x, 3)
    x_hat = np.array([
        [0.0, -x[2], x[1]],
        [x[2], 0.0, -x[0]],
        [-x[1], x[0], 0.0]
    ])
    
    return x_hat


def vee(x):
    """Returns the vee map of a given 3x3 matrix.

    Args:
        x: (3x3 numpy array) hat of the input vector

    Returns:
        (3x1 numpy array) vee map of the input matrix
    """
    ensure_skew(x, 3)
    return np.array([x[2,1], x[0,2], x[1,0]])


def q_to_R(q):
    """Converts a quaternion of a rotation matrix in SO(3).

    Args:
        q: (4x1 numpy array) quaternion in [x, y, z, w] format

    Returns:
        R: (3x3 numpy array) rotation matrix corresponding to the quaternion
    """

    # TODO: ensure quaternion instead of ensure vector
    ensure_vector(q, 4)

    R = np.identity(3)
    q13 = np.array([q[0], q[1], q[2]])
    q4 = q[3]

    hat_q = hat(q13)
    R += 2.0 * q4 * hat_q + 2.0 * hat_q.dot(hat_q)

    return R


def deriv_unit_vector(A, A_dot, A_2dot):
    """Returns the unit vector and it's derivatives for a given vector.

    Args:
        A: (3x1 numpy array) vector
        A_dot: (3x1 numpy array) first derivative of the vector
        A_2dot: (3x1 numpy array) second derivative of the vector

    Returns:
        q: (3x1 numpy array) unit vector of A
        q_dot: (3x1 numpy array) first derivative of q
        q_2dot: (3x1 numpy array) second derivative of q
    """

    ensure_vector(A, 3)
    ensure_vector(A_dot, 3)
    ensure_vector(A_2dot, 3)

    nA = np.linalg.norm(A)

    if abs(np.linalg.norm(nA)) < 1.0e-9:
        raise ZeroDivisionError('The 2-norm of A should not be zero')

    nA3 = nA * nA * nA
    nA5 = nA3 * nA * nA

    A_A_dot = A.dot(A_dot)

    q = A / nA
    q_dot = A_dot / nA \
        - A.dot(A_A_dot) / nA3

    q_2dot = A_2dot / nA \
        - A_dot.dot(2.0 * A_A_dot) / nA3 \
        - A.dot(A_dot.dot(A_dot) + A.dot(A_2dot)) / nA3 \
        + 3.0 * A.dot(A_A_dot).dot(A_A_dot)  / nA5

    return (q, q_dot, q_2dot)


def saturate(x, x_min,  x_max):
    """Saturate input vector between two values.
    
    Args:
        x: (nx1 numpy ndarray) value
        x_min: (float) minimum value for x
        x_max: (float) maximum value for x
    
    Returns:
        (nx1 numpy ndarray) saturated x
    """

    for i in range(len(x)):
        if x[i] > x_max:
            x[i] = x_max
        elif x[i] < x_min:
            x[i] = x_min
    
    return x


def expm_SO3(r):
    """Returns the matrix exponential of a matrix in SO(3).

    Args:
        r: (3x1 numpy array) vector

    Returns:
        R: (3x3 numpy array) matrix exponential of r
    """

    ensure_vector(r, 3)

    theta = np.linalg.norm(r)

    y = sinx_over_x(theta)
    y2 = sinx_over_x(theta / 2.0)

    hat_r = hat(r)
    R = np.eye(3) + y * hat_r + 0.5 * y2**2 * hat_r @ hat_r

    return R


def sinx_over_x(x):
    """Calculate sin(x)/x, while dealing with the cases where denominator is 
    zero.

    Args:
        x: (float) value

    Returns:
        y: (float) value of sin(x)/x
    """
    
    eps = 1e-6
    if abs(x) < eps:
        y = - x**10 / 39916800.0 + x**8 / 362880.0 - x**6 / 5040.0 \
            + x**4 / 120.0 - x**2 / 6.0 + 1.0
    else:
        y = np.sin(x) / x
    
    return y


def ensure_vector(x, n):
    """Make sure the given input array x is a vector of size n.

    Args:
        x: (nx1 numpy array) vector
        n: (int) desired length of the vector

    Returns:
        True if the input array is satisfied with size constraint. Raises an
        exception otherwise.
    """

    np.atleast_2d(x)  # Make sure the array is atleast 2D.

    if not len(np.ravel(x)) == n:
        raise ValueError('Input array needs to be of length {}, but the size' \
            'detected is {}'.format(n, np.shape(x)))
    
    return True


def ensure_matrix(x, m, n):
    """Make sure the given input array x is a matrix of size mxn.

    Args:
        x: (mxn numpy array) array
        m: (int) desired number of rows
        n: (int) desired number of columns

    Returns:
        True if the input array is satisfied with size constraint. Raises an
        exception otherwise.
    """

    np.atleast_2d(x)  # Make sure the array is atleast 2D.

    if not np.shape(x) == (m, n):
        raise ValueError('Input array needs to be of size {} x {}, but the ' \
            'size detected is {}'.format(m, n, np.shape(x)))
    
    return True


def ensure_skew(x, n):
    """Make sure the given input array is a skew-symmetric matrix of size nxn.

    Args:
        x: (nxn numpy array) array
        m: (int) desired number of rows and columns

    Returns:
        True if the input array is a skew-symmetric matrix. Raises an
        exception otherwise.
    """
    ensure_matrix(x, n, n)
    
    if not np.allclose(x.T, -x):
        raise ValueError('Input array must be a skew-symmetric matrix')
    
    return True


def ensure_SO3(x):
    """Make sure the given input array is in SO(3).

    Args:
        x: (3x3 numpy array) matrix

    Returns:
        True if the input array is in SO(3). Raises an exception otherwise.
    """
    ensure_matrix(x, 3, 3)
    
    if not np.array_equal(x.T @ x, np.eye(3)):
        raise ValueError('Input array does not satisfy R^TR = I')

    if not abs(np.linalg.det(x)) - 1.0 < 1e-9:
        raise ValueError('Input array does not det(R) = 1')
    
    return True