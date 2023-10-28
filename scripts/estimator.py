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

        # Get the time difference between two loops
        h = self.get_dt()

        # Save the current state estimates and accelerometer bias estimate as the previous estimates
        self.R_pre = np.copy(self.R)
        self.W_pre = np.copy(self.W)
        self.b_a_pre = self.b_a * 1.0

        # Update the angular rate estimate and attitude estimate using the IMU measurements
        self.W = self.R_bi.dot(W_imu)
        self.R = self.R.dot(expm_SO3(h / 2.0 * (self.W + self.W_pre)))

        # Update the acceleration estimate using the IMU measurements and the previous accelerometer bias estimate
        # This assumes IMU provide acceleration without g
        self.a = self.R.dot(self.R_bi).dot(a_imu) + self.b_a * self.e3
        a_pre = self.R_pre.dot(self.R_bi).dot(self.a_imu_pre) \
            + self.b_a_pre * self.e3

        # Update the position and velocity estimates using the acceleration estimate
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

        # Calculate A(t_k) and F(t_k)
        A = self.eye10 + h * A.dot(psi)
        F = h * psi.dot(F)

        # Update the error covariance matrix P using A(t_k) and F(t_k)
        self.P = A.dot(self.P).dot(A.T) + F.dot(self.Q).dot(F.T)

        # Save the current acceleration measurement as the previous measurement
        self.a_imu_pre = a_imu


    def imu_correction(self, R_imu, V_R_imu):
        """IMU correction step of the estimator.

        Args:
            R_imu: (3x3 numpy array) attitude measured by the IMU in SO(3)
            V_R_imu: (3x3 numpy array) attitude measurement covariance
        """

        # Calculate the attitude error between the IMU measurement and the current attitude estimate
        imu_R = self.R.T.dot(R_imu).dot(self.R_bi_T)
        del_z = 0.5 * vee(imu_R - imu_R.T)

        # Construct the measurement matrix H and its transpose
        H = np.block([self.zero3, self.zero3, self.eye3, np.zeros((3, 1))])
        H_T = H.T

        # Construct the gain matrix G and its transpose
        G = self.R_bi
        G_T = G.T

        # Construct the measurement covariance matrix V
        V = V_R_imu

        # Calculate the innovation covariance matrix S, the Kalman gain K, and the state correction X
        S = H.dot(self.P).dot(H_T) + G.dot(V).dot(G_T)
        K = self.P.dot(H_T).dot(np.linalg.inv(S))
        X = K.dot(del_z)

        # Extract the attitude correction from X and update the attitude estimate
        eta = X[6:9]
        self.R = self.R.dot(expm_SO3(eta))

        # Update the error covariance matrix P using the Kalman gain
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

        # Calculate the difference between the GPS measurements and the current state estimates
        del_z = np.hstack((x_gps - self.x, v_gps - self.v))

        # Construct the measurement matrix H and its transpose
        H = np.block([
            [self.eye3, self.zero3, self.zero3, np.zeros((3, 1))],
            [self.zero3, self.eye3, self.zero3, np.zeros((3, 1))]
        ])
        H_T = H.T

        # Construct the measurement covariance matrix V
        V = np.block([
            [V_x_gps, self.zero3],
            [self.zero3, V_v_gps]
        ])

        # Calculate the innovation covariance matrix S, the Kalman gain K, and the state correction X
        S = H.dot(self.P).dot(H_T) + V
        K = self.P.dot(H_T).dot(np.linalg.inv(S))
        X = K.dot(del_z)

        # Extract the position correction, velocity correction, and accelerometer bias correction from X
        dx = X[0:3]
        dv = X[3:6]
        db_a = X[9]

        # Update the position, velocity, and accelerometer bias estimates using the state correction
        self.x = self.x + dx
        self.v = self.v + dv
        self.b_a = self.b_a + db_a

        # Update the error covariance matrix P using the Kalman gain
        I_KH = self.eye10 - K.dot(H)
        self.P = I_KH.dot(self.P).dot(I_KH.T) + K.dot(V).dot(K.T)

    
    def get_dt(self):
        """Get the time difference between two loops.

        Return:
            dt: (float) time difference between two loops
        """

        # Save the current time as the previous time and get the current time
        self.t_pre = self.t * 1.0
        t_now = datetime.datetime.now()

        # Calculate the time difference between the current time and the start time
        self.t = (t_now - self.t0).total_seconds()

        # Return the time difference between the current time and the previous time
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
