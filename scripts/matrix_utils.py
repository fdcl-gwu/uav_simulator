import numpy as np
import pdb

def hat(x):
    
    x = x.reshape((3, 1))
    x_hat = np.array([
        [0.0, -x[2], x[1]],
        [x[2], 0.0, -x[0]],
        [-x[1], x[0], 0.0]
    ])
    
    return x_hat


def vee(x):
    return np.array([x[2,1], x[0,2], x[1,0]])


def q_to_R(q):
    R = np.identity(3)
    q13 = np.array([q[0], q[1], q[2]])
    q4 = q[3]

    hat_q = hat(q13)
    R += 2 * q4 * hat_q + 2 * hat_q.dot(hat_q)

    return R


def deriv_unit_vector(A, A_dot, A_2dot):

    nA = np.linalg.norm(A)
    nA3 = nA * nA * nA
    nA5 = nA3 * nA * nA

    A_A_dot = A.dot(A_dot)

    q = A / nA
    q_dot = A_dot / nA \
        - A.dot(A_A_dot) / nA3

    q_2dot = A_2dot / nA \
        - A_dot.dot(2 * A_A_dot) / nA3 \
        - A.dot(A_dot.dot(A_dot) + A.dot(A_2dot)) / nA3 \
        + 3 * A.dot(A_A_dot).dot(A_A_dot)  / nA5

    return (q, q_dot, q_2dot)


def saturate(x, x_min,  x_max):

    for i in range(len(x)):
        if x[i] > x_max:
            x[i] = x_max
        elif x[i] < x_min:
            x[i] = x_min
    
    return x


def expm_SO3(r):
    theta = np.linalg.norm(r)

    y = sinx_over_x(theta)
    y2 = sinx_over_x(theta / 2)

    R = np.eye(3) + y * hat(r) + 0.5 * y2**2 * hat(r)**2

    return R


def sinx_over_x(x):
    eps = 1e-6
    if abs(x) < eps:
        y = - x**10 / 39916800.0 + x**8 / 362880.0 - x**6 / 5040.0 \
            + x**4 / 120.0 - x**2 / 6.0 + 1.0
    else:
        y = np.sin(x) / x
    
    return y
