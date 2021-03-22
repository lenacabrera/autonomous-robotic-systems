import numpy as np
import math


class KalmanFilter:
    def __init__(self, x, y, Theta, delta_t):

        # TODO: in beginning, should trajectory move if robot has not moved?

        # CONFIGURATION
        # randomly drawing from normal distribution (Gaussian)
        self.mean = 1  # 0.1
        self.std_Sigma = 0.01  # 0.04
        self.std_R = 0.01
        self.std_Q = 0.01
        self.std_z = 0.01
        self.uncertainty_growth = 0.1

        # state vector / robot belief
        self.mu = np.array([[x],
                            [y],
                            [Theta]])

        # covariance / uncertainty about belief
        self.Sigma = np.array([[np.random.normal(self.mean, self.std_Sigma), 0, 0],
                               [0, np.random.normal(self.mean, self.std_Sigma), 0],
                               [0, 0, np.random.normal(self.mean, self.std_Sigma)]])

        """" Motion model """
        # state transition matrix
        self.A = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])

        # control transition matrix
        self.B = np.array([[delta_t * math.cos(self.mu[2]), 0],
                           [delta_t * math.cos(self.mu[2]), 0],
                           [0, delta_t]])

        # process noise (Gaussian)
        self.R = np.array([[np.random.normal(self.mean, self.std_R), 0, 0],
                           [0, np.random.normal(self.mean, self.std_R), 0],
                           [0, 0, np.random.normal(self.mean, self.std_R)]])

        # interim uncertainty (after motion, before correction)
        self.Sigma_estimate = self.Sigma

        """" Sensor model """
        # observation transition matrix
        self.C = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])

        # measurement noise
        self.Q = np.array([[np.random.normal(self.mean, self.std_Q), 0, 0],
                           [0, np.random.normal(self.mean, self.std_Q), 0],
                           [0, 0, np.random.normal(self.mean, self.std_Q)]])

        self.I = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])

        # belief trajectory
        self.positions = [(x, y)]

    def kalman_filter_update(self, v, omega, visible_landmarks, distances, bearings, increased_uncertainty):

        # linearly growing uncertainty when not three landmarks visible
        add_uncertainty = self.uncertainty_growth * increased_uncertainty
        self.Sigma = np.matmul(self.Sigma, np.array([[add_uncertainty, 0, 0],
                                                     [0, add_uncertainty, 0],
                                                     [0, 0, add_uncertainty]]))

        # control vector
        u = np.array([[v],
                      [omega]])

        """ Prediction """
        # state transition to get state estimate
        mu_estimate = np.matmul(self.A, self.mu) + np.matmul(self.B, u)
        # uncertainty introduced by state transition
        self.Sigma_estimate = np.matmul(np.matmul(self.A, self.Sigma), self.A.T) + self.R

        # measurement from sensors
        z = self.estimate_z(mu_estimate, visible_landmarks, distances, bearings)

        """ Correction """
        # Kalman gain -> degree to which measurement (z) is incorporated
        K = np.matmul(np.matmul(self.Sigma_estimate, self.C.T),
                      np.linalg.inv(np.matmul(np.matmul(self.C, self.Sigma_estimate), self.C.T) + self.Q))
        # update state
        self.mu = mu_estimate + np.matmul(K, (z - np.matmul(self.C, mu_estimate)))
        # update uncertainty
        self.Sigma = np.matmul((self.I - np.matmul(K, self.C)), self.Sigma_estimate)

        # log positions to draw path trajectory
        self.positions.append((self.mu[0][0], self.mu[1][0]))

    def estimate_z(self, mu_estimate, visible_landmarks, distances, bearings):
        """"
        1. new position (x, y) estimate
            a. trilateration to find intersection point of three circles (landmark position + distance)
        2. new orientation (theta) estimate
            a. calculate bearing of one landmark
            b. update orientation according
        3. create z from 1. and 2.
        """

        if len(visible_landmarks) == 3:

            # apply trilateration formulas to return the (x,y) intersection point of three circles
            # z is estimation of position based on sensor info/distances

            x1 = visible_landmarks[0].x
            y1 = visible_landmarks[0].y
            r1 = distances[0]
            x2 = visible_landmarks[1].x
            y2 = visible_landmarks[1].y
            r2 = distances[1]
            x3 = visible_landmarks[2].x
            y3 = visible_landmarks[2].y
            r3 = distances[2]

            A = 2 * x2 - 2 * x1
            B = 2 * y2 - 2 * y1
            C = r1 ** 2 - r2 ** 2 - x1 ** 2 + x2 ** 2 - y1 ** 2 + y2 ** 2
            D = 2 * x3 - 2 * x2
            E = 2 * y3 - 2 * y2
            F = r2 ** 2 - r3 ** 2 - x2 ** 2 + x3 ** 2 - y2 ** 2 + y3 ** 2
            x = (C * E - F * B) / (E * A - B * D)
            y = (C * D - A * F) / (B * D - A * E)

            # average of bearing
            bearing = sum(bearings) / len(bearings)

        else:
            x = mu_estimate[0][0]
            y = mu_estimate[1][0]
            bearing = mu_estimate[2][0]

        # add noise to mimic sensor noise
        x_noise = np.random.normal(self.mean, self.std_z)
        y_noise = np.random.normal(self.mean, self.std_z)
        b_noise = np.random.normal(self.mean, self.std_z)
        z = np.array([[x + x_noise],
                      [y + y_noise],
                      [bearing + b_noise]])

        return z
