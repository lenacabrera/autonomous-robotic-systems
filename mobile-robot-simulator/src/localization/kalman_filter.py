import numpy as np
import math


class KalmanFilter:

    def __init__(self, conf, x, y, theta):

        # configuration parameters
        self.mean = conf.kf_mean
        self.std_Sigma = conf.kf_std_Sigma
        self.std_R = conf.kf_std_R
        self.std_Q = conf.kf_std_Q
        self.std_z = conf.kf_std_z
        self.uncertainty_growth = conf.kf_uncertainty_growth

        # state vector / robot belief
        self.mu = np.array([[x],
                            [y],
                            [theta]])

        # covariance / uncertainty about belief
        self.Sigma = np.array([[np.random.normal(0, self.std_Sigma), 0, 0],
                               [0, np.random.normal(0, self.std_Sigma), 0],
                               [0, 0, np.random.normal(0, self.std_Sigma)]])

        """" Motion model """
        # state transition matrix
        self.A = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 0, 1]])

        # process noise (Gaussian)
        self.R = np.array([[np.random.normal(self.mean, self.std_R), 0, 0],
                           [0, np.random.normal(self.mean, self.std_R), 0],
                           [0, 0, np.random.normal(self.mean, self.std_R)]])

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

        # intermediate uncertainties
        self.uncertainty_history = []


    def update_belief(self, robot, delta_t, visible_landmarks, distances, bearings, increased_uncertainty):
        """ Performs update of robot's belief about its pose (position and orientation) """

        v = (robot.v_wheel_l + robot.v_wheel_r) / 2
        omega = robot.omega

        # linearly growing uncertainty when not three landmarks visible
        additional_uncertainty = self.uncertainty_growth * increased_uncertainty
        self.Sigma = np.matmul(self.Sigma, np.array([[additional_uncertainty, 0, 0],
                                                     [0, additional_uncertainty, 0],
                                                     [0, 0, 0]]))

        # control transition matrix
        B = np.array([[delta_t * math.cos(self.mu[2][0]), 0],
                      [delta_t * math.sin(self.mu[2][0]), 0],
                      [0, delta_t]])

        # control vector
        u = np.array([[v],
                      [omega]])

        """ Prediction """
        # state transition to get state estimate
        mu_estimate = np.matmul(self.A, self.mu) + np.matmul(B, u)
        # uncertainty introduced by state transition
        Sigma_estimate = np.matmul(np.matmul(self.A, self.Sigma), self.A.T) + self.R

        """ Correction """
        # Kalman gain -> degree to which measurement (z) is incorporated
        K = np.matmul(np.matmul(Sigma_estimate, self.C.T),
                      np.linalg.inv(np.matmul(np.matmul(self.C, Sigma_estimate), self.C.T) + self.Q))
        # measurement from sensors
        z = self.estimate_z(visible_landmarks, distances, bearings, v, omega, mu_estimate, self.mu)
        # update state
        self.mu = mu_estimate + np.matmul(K, (z - np.matmul(self.C, mu_estimate)))
        # update uncertainty
        self.Sigma = np.matmul((self.I - np.matmul(K, self.C)), Sigma_estimate)

        # log positions to draw path trajectory
        self.positions.append((self.mu[0][0], self.mu[1][0]))

        # log interim uncertainty (after motion, before correction) for visualization -> (x, y, width, height)
        self.uncertainty_history.append((self.mu[0][0], self.mu[1][0],
                                         abs(Sigma_estimate[0][0]) * 10, abs(Sigma_estimate[1][1]) * 10))


    def estimate_z(self, visible_landmarks, distances, bearings, v, omega, mu_estimate, mu):
        """ Corrects belief (if 3 landmarks are measured, else no correction)
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
            theta = sum(bearings) / len(bearings)

        else:
            # no correction
            x = mu[0][0]
            y = mu[1][0]
            theta = mu[2][0]

            m1 = np.array([[x],
                           [y],
                           [theta]
                           ])

            m2 = np.array([[1 * np.cos(theta), 0],
                           [1 * np.sin(theta), 0],
                           [0, 1]
                           ])

            m3 = np.array([[v],
                           [omega],
                           ])

            new = m1 + np.matmul(m2, m3)

            x = mu_estimate[0][0]
            y = mu_estimate[1][0]
            theta = new[2][0]

        # add noise to mimic sensor noise
        x_noise = np.random.normal(0, self.std_z)
        y_noise = np.random.normal(0, self.std_z)
        t_noise = np.random.normal(0, self.std_z)
        z = np.array([[x + x_noise],
                      [y + y_noise],
                      [theta + t_noise]])

        return z
