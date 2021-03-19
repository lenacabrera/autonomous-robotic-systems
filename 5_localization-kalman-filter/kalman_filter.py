import numpy as np
import math
import random
from shapely.geometry import Point


class KalmanFilter:
    def __init__(self, x, y, Theta, v, omega):
        # slide 9
        self.mu = np.array([[x],
                            [y],
                            [Theta]])

        # slide 9 # Robot sure about starting point (TODO maybe make random)
        self.Sigma = np.array([[0.47893, 0, 0],
                               [0, 0.92875, 0],
                               [0, 0, 0.12378]])

        self.Sigma_estimate = np.array([[0, 0, 0],
                                        [0, 0, 0],
                                        [0, 0, 0]])

        self.positions = [(x, y)]


    def kalman_filter_call(self, v, omega, delta_t, visible_landmarks, distances, bearings):
        u = np.array([[v],
                      [omega]])

        z = self.estimate_z(visible_landmarks, distances, bearings)

        self.kalman_filter_update(u, z, delta_t)

    def kalman_filter_update(self, u, z, delta_t):
        # slide 15
        A = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        # slide 15
        B = np.array([[delta_t * math.cos(self.mu[2]), 0],
                      [delta_t * math.cos(self.mu[2]), 0],
                      [0, delta_t]])

        # slide 16 # TODO
        R = np.array([[np.random.uniform(0,1), 0, 0],
                      [0, np.random.uniform(0,1), 0],
                      [0, 0, np.random.uniform(0,1)]])

        # slide 26
        C = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        # slide 27 # TODO
        Q = np.array([[np.random.uniform(0,1), 0, 0],
                      [0, np.random.uniform(0,1), 0],
                      [0, 0, np.random.uniform(0,1)]])

        I = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        # prediction
        mu_estimate = np.matmul(A, self.mu) + np.matmul(B, u)
        self.Sigma_estimate = np.matmul(np.matmul(A, self.Sigma), np.transpose(A)) + R

        # correction
        K = np.matmul(np.matmul(self.Sigma_estimate, np.transpose(C)),
                      np.linalg.inv(np.matmul(np.matmul(C, self.Sigma_estimate), np.transpose(C)) + Q))
        print("K", K)

        self.mu = mu_estimate + np.matmul(K, (z - np.matmul(C, mu_estimate)))
        self.Sigma = np.matmul((I - np.matmul(K, C)), self.Sigma_estimate)
        print("Sigma", self.Sigma)

        self.positions.append((self.mu[0][0], self.mu[1][0]))

    def estimate_z(self, visible_landmarks, distances, bearings):
        """"
        1. new position (x, y) estimate
        a. calculate distances to visible landmarks
        b. per landmark take coordinates and distance for triangulation

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

            bearing_avg = sum(bearings) / len(bearings)
            # add noise to mimic sensor noise
            x_noise = 0.001
            y_noise = 0.002
            b_noise = 0.003
            z = np.array([[x + x_noise],
                          [y + y_noise],
                          [bearing_avg + b_noise]])

        else:
            z = self.mu

        return z
