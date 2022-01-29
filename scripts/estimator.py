from matrix_utils import hat, vee, expm_SO3

import datetime
import numpy as np


class Estimator:
    """Estimates the states of the UAV.

    This uses the estimator defined in "Real-time Kinematics GPS Based 
    Telemetry System for Airborne Measurements of Ship Air Wake", but without
    the bias estimation terms.
    DOI: 10.2514/6.2019-2377

    x (3x1 numpy array) current position of the UAV [m]
    x: (3x1 numpy array) current position of the UAV [m]
    v: (3x1 numpy array) current velocity of the UAV [m/s]
    a: (3x1 numpy array) current acceleration of the UAV [m/s^s]
    b_a: (float) accelerometer bias in e3 direction [m/s^2]
    R: (3x3 numpy array) current attitude of the UAV in SO(3)
    W: (3x1 numpy array) current angular velocity of the UAV [rad/s]
    
    Q: (7x7 numpy array) variances of w_k
    P: (10x10 numpy array) covariances of the states

    t0: (datetime object) time at epoch
    t: (float) current time since epoch [s]
    t_prev: (float) time since epoch in the previous loop [s]

    W_pre: (3x1 numpy array) angular velocity of the previous loop [rad/s]
    a_imu_pre: (3x1 numpy array) acceleration of the previous loop [m/s^2]
    R_pre: (3x3 numpy array) attitude in the previous loop in SO(3)
    b_a_pre: (3x1 numpy array) accelerometer bias in the previous loop [m/s^2]

    g: (float) gravitational acceleration [m/s^2]
    ge3: (3x1 numpy array) gravitational acceleration direction [m/s^2]

    R_bi: (3x3 numpy array) transformation from IMU frame to the body frame
    R_bi_T: (3x3 numpy array) transformation from IMU frame to the body frame

    e3 : (3x1 numpy array) direction of the e3 axis
    eye3: (3x3 numpy array) 3x3 identity matrix
    eye10: (10x10 numpy array) 10x10 identity matrix

    zero3: (3x3 numpy array) 3x3 zero matrix
    """

    def __init__(self):
        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.b_a = 0.0
        self.R = np.eye(3)
        self.W = np.zeros(3)
        
        # Variances of w_k
        self.Q = np.diag([
            0.001, 0.001, 0.001,  # acceleration
            0.025, 0.025, 0.025,  # angular velocity
            0.0001  # acclerometer z bias
        ])

        # Initial covariances of x
        self.P = np.diag([
            1.0, 1.0, 1.0,  # position
            1.0, 1.0, 1.0,  # velocity
            0.01, 0.01, 0.01,  # attitude
            1.0  # accelerometer z bias
        ])

        self.t0 = datetime.datetime.now()
        self.t = 0.0
        self.t_pre = 0.0

        self.W_pre = np.zeros(3)
        self.a_imu_pre = np.zeros(3)
        self.R_pre = np.eye(3)
        self.b_a_pre = 0.0

        self.g = 9.81
        self.ge3 = np.array([0.0, 0.0, self.g])

        # Transformation from IMU frame to the body frame.
        self.R_bi = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])
        self.R_bi_T = self.R_bi.T

        self.e3 = np.array([0.0, 0.0, 1.0])
        self.eye3 = np.eye(3)
        self.eye10 = np.eye(10)

        self.zero3 = np.zeros((3, 3))


    def prediction(self, a_imu, W_imu):
        """Prediction step of the estimator.

        Args:
            a_imu: (3x1 numpy array) acceleration measured by the IMU [m/s^2]
            W_imu: (3x1 numpy array) angular rate measured by the IMU [rad/s]
        """

        h = self.get_dt()

        self.R_pre = np.copy(self.R)
        self.W_pre = np.copy(self.W)
        self.b_a_pre = self.b_a * 1.0

        self.W = self.R_bi.dot(W_imu)
        self.R = self.R.dot(expm_SO3(h / 2.0 * (self.W + self.W_pre)))

        # This assumes IMU provide acceleration without g
        self.a = self.R.dot(self.R_bi).dot(a_imu) + self.b_a * self.e3
        a_pre = self.R_pre.dot(self.R_bi).dot(self.a_imu_pre) \
            + self.b_a_pre * self.e3

        self.x = self.x + h * self.v + h**2 / 2.0 * a_pre
        self.v = self.v + h / 2.0 * (self.a + a_pre)

        # Calculate A(t_{k-1})
        A = np.zeros((10, 10))
        A[0:3, 3:6] = self.eye3
        A[3:6, 6:9] = - self.R_pre.dot(hat(self.R_bi.dot(self.a_imu_pre)))
        A[3:6, 9] = self.e3
        A[6:9, 6:9] = -hat(self.R_bi.dot(W_imu))

        # Calculate F(t_{k-1})
        F = np.zeros((10, 7))
        F[3:6, 0:3] = self.R_pre.dot(self.R_bi)
        F[6:9, 3:6] = self.R_bi
        F[9, 6] = 1.0

        # Calculate \Psi using A(t)
        psi = self.eye10 + h / 2.0 * A

        A = self.eye10 + h * A.dot(psi)
        F = h * psi.dot(F)

        self.P = A.dot(self.P).dot(A.T) + F.dot(self.Q).dot(F.T)

        self.a_imu_pre = a_imu


    def imu_correction(self, R_imu, V_R_imu):
        """IMU correction step of the estimator.

        Args:
            R_imu: (3x3 numpy array) attitude measured by the IMU in SO(3)
            V_R_imu: (3x3 numpy array) attitude measurement covariance
        """

        imu_R = self.R.T.dot(R_imu).dot(self.R_bi_T)
        del_z = 0.5 * vee(imu_R - imu_R.T)

        H = np.block([self.zero3, self.zero3, self.eye3, np.zeros((3, 1))])
        H_T = H.T

        G = self.R_bi
        G_T = G.T

        V = V_R_imu

        S = H.dot(self.P).dot(H_T) + G.dot(V).dot(G_T)
        K = self.P.dot(H_T).dot(np.linalg.inv(S))

        X = K.dot(del_z)

        eta = X[6:9]
        self.R = self.R.dot(expm_SO3(eta))

        I_KH = self.eye10 - K.dot(H)
        self.P = I_KH.dot(self.P).dot(I_KH.T) \
            + K.dot(G).dot(V).dot(G_T).dot(K.T)


    def gps_correction(self, x_gps, v_gps, V_x_gps, V_v_gps):
        """GPS correction step of the estimator.

        Args:
            x_gps: (3x1 numpy array) position measured by the GPS [m]
            v_gps: (3x1 numpy array) velocity measured by the GPS [m]
            V_x_gps: (3x1 numpy array) position measurement covariance
            V_v_gps: (3x1 numpy array) velocity measurement covariance
        """

        del_z = np.hstack((x_gps - self.x, v_gps - self.v))

        H = np.block([
            [self.eye3, self.zero3, self.zero3, np.zeros((3, 1))],
            [self.zero3, self.eye3, self.zero3, np.zeros((3, 1))]
        ])
        H_T = H.T

        V = np.block([
            [V_x_gps, self.zero3],
            [self.zero3, V_v_gps]
        ])

        S = H.dot(self.P).dot(H_T) + V
        K = self.P.dot(H_T).dot(np.linalg.inv(S))

        X = K.dot(del_z)

        dx = X[0:3]
        dv = X[3:6]
        db_a = X[9]

        self.x = self.x + dx
        self.v = self.v + dv
        self.b_a = self.b_a + db_a

        I_KH = self.eye10 - K.dot(H)
        self.P = I_KH.dot(self.P).dot(I_KH.T) + K.dot(V).dot(K.T)

    
    def get_dt(self):
        """Get the time difference between two loops.

        Return:
            dt: (float) time difference between two loops
        """

        self.t_pre = self.t * 1.0
        t_now = datetime.datetime.now()
        self.t = (t_now - self.t0).total_seconds()

        return self.t - self.t_pre


    def get_states(self):
        """Return the current states of the estimator.

        Return:
            x: (3x1 numpy array) current position of the UAV [m]
            v: (3x1 numpy array) current velocity of the UAV [m/s]
            a: (3x1 numpy array) current acceleration of the UAV [m/s^s]
            R: (3x3 numpy array) current attitude of the UAV in SO(3)
            W: (3x1 numpy array) current angular velocity of the UAV [rad/s]
        """
        return (self.x, self.v, self.a, self.R, self.W)
