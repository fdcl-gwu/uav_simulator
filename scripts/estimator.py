from matrix_utils import hat, vee, expm_SO3

import datetime
import numpy as np


class Estimator:
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
        h = self.get_dt()

        self.R_pre = np.copy(self.R)
        self.W_pre = np.copy(self.W)
        self.b_a_pre = self.b_a * 1.0

        self.W = self.R_bi.dot(W_imu)
        self.R = self.R.dot(expm_SO3(h / 2 * (self.W + self.W_pre)))

        # This assumes IMU provide acceleration without g
        self.a = self.R.dot(self.R_bi).dot(a_imu) + self.b_a * self.e3
        a_pre = self.R_pre.dot(self.R_bi).dot(self.a_imu_pre) \
            + self.b_a_pre * self.e3

        self.x = self.x + h * self.v + h**2 / 2 * a_pre
        self.v = self.v + h / 2 * (self.a + a_pre)

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
        psi = self.eye10 + h / 2 * A

        A = self.eye10 + h * A.dot(psi)
        F = h * psi.dot(F)

        self.P = A.dot(self.P).dot(A.T) + F.dot(self.Q).dot(F.T)

        self.a_imu_pre = a_imu


    def imu_correction(self, R_imu, V_R_imu):
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
        self.t_pre = self.t * 1.0
        t_now = datetime.datetime.now()
        self.t = (t_now - self.t0).total_seconds()

        return self.t - self.t_pre


    def get_states(self):
        return (self.x, self.v, self.a, self.R, self.W)
