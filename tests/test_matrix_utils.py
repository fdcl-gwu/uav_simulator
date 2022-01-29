import imp
import numpy as np
import unittest

from scripts.matrix_utils import *


class TestMatrixUtils(unittest.TestCase):

    def setUp(self):
        self.x = np.array([1.0, 2.0, 3.0])
        self.x_hat = np.array([
            [0.0, -3.0, 2.0],
            [3.0, 0.0, -1.0],
            [-2.0, 1.0, 0.0]
        ])

        theta = np.pi / 2.0
        self.R = np.array([
            [np.cos(theta), np.sin(theta), 0.0],
            [-np.sin(theta), np.cos(theta), 0.0],
            [0.0, 0.0, 1.0]
        ])

        self.q1 = np.array([0.7071068, 0.0, 0.0, 0.7071068])
        self.R_from_q1 = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0]
        ])

        self.q2 = np.array([0.033166, -0.1902412, 0.264125, 0.9449584])
        self.R_from_q2 = np.array([
            [0.7880926, -0.5117933, -0.3420202],
            [0.4865551,  0.8582760, -0.1631759],
            [0.3770600, -0.0378139,  0.9254166]
        ])

        self.A = np.array([1.0, 0.0, 0.0])
        self.A_dot = np.array([0.0, 0.0, 0.0])
        self.A_2dot = np.array([0.0, 0.0, 0.0])

    ## ensure_vector
    def test_ensure_vector(self):
        self.assertTrue(ensure_vector(self.x, 3))

    def test_ensure_vector_wrong_input_length(self):
        self.assertRaises(ValueError, ensure_vector, self.x, 2)

    def test_ensure_vector_matrix_input(self):
        self.assertRaises(ValueError, ensure_vector, self.x_hat, 3)

    def test_ensure_vector_3d_matrix_input(self):
        self.assertRaises(ValueError, ensure_vector, np.zeros((3, 3, 3)), 3)

    def test_ensure_vector_list_input(self):
        self.assertTrue(ensure_vector([0, 1, 2], 3))

    ## ensure_matrix
    def test_ensure_matrix(self):
        self.assertTrue(ensure_matrix(self.x_hat, 3, 3))

    def test_ensure_matrix_2d_input(self):
        self.assertRaises(ValueError, ensure_matrix, self.x, 3, 3)
    
    def test_ensure_matrix_3d_input(self):
        self.assertRaises(ValueError, ensure_matrix, np.zeros((3, 3, 3)), 3, 3)

    ## ensure_skew
    def test_ensure_skew(self):
        self.assertTrue(ensure_skew(self.x_hat, 3))

    def test_ensure_skew_nonskew_input(self):
        self.assertRaises(ValueError, ensure_skew, np.eye((3)), 3)

    def test_ensure_skew_3d_matrix(self):
        self.assertRaises(ValueError, ensure_skew, np.zeros((3, 3, 3)), 3)

    ## ensure_SO3
    def test_ensure_SO3(self):
        self.assertTrue(ensure_SO3(self.R))

    def test_ensure_SO3_eye(self):
        self.assertTrue(ensure_SO3(np.eye(3)))

    def test_ensure_SO3_wrong_R(self):
        R = self.R.copy()
        R[2, 2] = 1.001
        self.assertRaises(ValueError, ensure_SO3, R)

    def test_ensure_SO3_array_input(self):
        self.assertRaises(ValueError, ensure_SO3, [1.0, 2.0, -10.0, 5.5])

    def test_ensure_SO3_3d_matrix_input(self):
        self.assertRaises(ValueError, ensure_SO3, np.zeros((3, 3, 3)))

    ## hat
    def test_hat(self):
        np.testing.assert_array_almost_equal(hat(self.x), self.x_hat)

    def test_hat_short_input(self):
        self.assertRaises(ValueError, hat, np.array([1.0, 2.0]))

    def test_hat_long_input(self):
        self.assertRaises(ValueError, hat, [1.0, 2.0, -10.0, 5.5])

    ## vee
    def test_vee(self):
        np.testing.assert_array_almost_equal(vee(self.x_hat), self.x)

    def test_vee_non_skew(self):
        self.assertRaises(ValueError, vee, np.eye(3))

    ## q_to_R
    def test_q_to_R_q1(self):
        np.testing.assert_array_almost_equal(q_to_R(self.q1), self.R_from_q1, 6)

    def test_q_to_R_q2(self):
        np.testing.assert_array_almost_equal(q_to_R(self.q2), self.R_from_q2, 6)

    def test_q_to_R_short_array(self):
        self.assertRaises(ValueError, ensure_SO3, [0.0, 1.0, 0.0])

    def test_q_to_R_long_array(self):
        self.assertRaises(ValueError, ensure_SO3, [0.0, 1.0, 0.0, 0.0, 1.0])

    def test_q_to_R_wrong_q(self):
        R = q_to_R([1.0, 1.0, 1.0, 1.0])
        self.assertRaises(ValueError, ensure_SO3, R)

    ## deriv_unit_vector
    def test_deriv_unit_vector(self):
        q, q_dot, q_2dot = deriv_unit_vector(self.A, self.A_dot, self.A_2dot)
        A_zero = np.array([0.0, 0.0, 0.0])
        self.assertTrue(
            np.array_equal(q, self.A) and
            np.array_equal(q_dot, A_zero) and
            np.array_equal(q_2dot, A_zero)
        )

    def test_deriv_unit_vector_non_zero(self):
        A = np.array([1.0, 2.0, 3.0])
        A_dot  = np.array([-1.2, -2.0, -4.5])
        A_2dot = np.array([0.1, 0.2, 1.7])
        q, q_dot, q_2dot = deriv_unit_vector(A, A_dot, A_2dot)

        A_q = np.array([0.26726124, 0.53452248, 0.80178373])
        A_q_dot = np.array([0.03627117, 0.17944683, -0.13172161])
        A_q_2dot = np.array([0.00312259, 0.29183291, -0.25903887])

        self.assertTrue(
            np.allclose(q, A_q) and
            np.allclose(q_dot, A_q_dot) and
            np.allclose(q_2dot, A_q_2dot)
        )

    def test_deriv_unit_vector_zero_norm_A(self):
        self.assertRaises(ZeroDivisionError, deriv_unit_vector, self.A_dot, \
            self.A_dot, self.A_dot)

    def test_deriv_unit_vector_short_A(self):
        self.assertRaises(ValueError, deriv_unit_vector, [0.0, 1.0], \
            self.A_dot, self.A_dot)

    def test_deriv_unit_vector_long_A(self):
        self.assertRaises(ValueError, deriv_unit_vector, self.q1, \
            self.A_dot, self.A_dot)

    def test_deriv_unit_vector_short_A_dot(self):
        self.assertRaises(ValueError, deriv_unit_vector, self.A_dot, \
            [0.0, 1.0], self.A_dot)

    def test_deriv_unit_vector_long_A_dot(self):
        self.assertRaises(ValueError, deriv_unit_vector, self.A, \
            self.q1, self.A_dot)

    def test_deriv_unit_vector_short_A_2dot(self):
        self.assertRaises(ValueError, deriv_unit_vector, self.A_dot, \
            self.A_dot, [0.0, 1.0])

    def test_deriv_unit_vector_long_A_2dot(self):
        self.assertRaises(ValueError, deriv_unit_vector, self.A, \
            self.A_dot, self.q1)

    ## Saturate
    def test_saturate(self):
        x = np.array([2.0, -6.0, 25.0])
        self.assertTrue(np.allclose(saturate(x, -5.0, 5.0), [2.0, -5.0, 5.0]))

    ## expm_SO3
    def test_expm_SO3(self):
        self.assertTrue(
            np.allclose(
                expm_SO3([1.0, 0.0, 0.0]), np.array([
                    [1.0, 0.0, 0.0],
                    [0.0, 0.54030230586814, -0.841470984807897],
                    [0.0, 0.841470984807897, 0.54030230586814]
                ])
            )
        )
    
    def test_expm_SO3_zero_inputs(self):
        self.assertTrue(np.allclose(expm_SO3([0.0, 0.0, 0.0]), np.eye(3)))
    
    def test_expm_SO3_nonzero_inputs(self):
        self.assertTrue(
            np.allclose(
                expm_SO3([2.3, -1.0, 3.3]), np.array([
                    [-0.0641039645213566, 0.465517438128782, 0.882714108038758],
                    [-0.87719769353654, -0.44804055054294, 0.172580043815485],
                    [0.475830734806843, -0.763251714617921, 0.43707199858374]
                ])
            )
        )
    
    def test_expm_SO3_short_input_array(self):
        self.assertRaises(ValueError, expm_SO3, [0.0, 0.0])
    
    def test_expm_SO3_long_input_array(self):
        self.assertRaises(ValueError, expm_SO3, [0.0, 0.0, 0.0, 0.0])
        
    ## sinx_over_x
    def test_sinx_over_x(self):
        self.assertTrue(np.isclose(sinx_over_x(np.pi / 3.0), 0.826993343132688))
    
    def test_sinx_over_x_pi(self):
        self.assertTrue(np.isclose(sinx_over_x(np.pi), 0.0))
    
    def test_sinx_over_x_zero(self):
        self.assertTrue(np.isclose(sinx_over_x(0.0), 1.0))


if __name__ == '__main__':
    unittest.main()
