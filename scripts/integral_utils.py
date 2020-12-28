import numpy as np


class IntegralErrorVec3:
    def __init__(self):
        self.error = np.zeros(3)
        self.integrand = np.zeros(3)


    def integrate(self, current_integrand, dt):
        self.error += (self.integrand + current_integrand) * dt / 2.0
        self.integrand = current_integrand

    def set_zero(self):
        self.error = np.zeros(3)
        self.integrand = np.zeros(3)


class IntegralError:
    def __init__(self):
        self.error = 0.0
        self.integrand = 0.0


    def integrate(self, current_integrand, dt):
        self.error += (self.integrand + current_integrand) * dt / 2.0
        self.integrand = current_integrand

    def set_zero(self):
        self.error = 0.0
        self.integrand = 0.0