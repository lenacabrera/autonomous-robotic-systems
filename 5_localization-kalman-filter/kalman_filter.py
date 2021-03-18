import numpy as np
import math
from shapely.geometry import Point

class KalmanFilter:
    def __init__(self, x, y, Theta, v, omega):
        # slide 9
        self.mu = np.array([[x],
                            [y],
                            [Theta]])

        # slide 9 # Robot sure about starting point (TODO maybe make random)
        self.Sigma = np.array([[0.0001, 0, 0],
                                [0, 0.0003, 0],
                                [0, 0, 0.0002]])

        self.positions = [(x, y)]

        # slide 15
        u = np.array([[v],
                      [omega]])

        # slide 26, TODO delta entspricht Q
        # z is estimation of position based on sensor info/distances
        delta = np.array([[0],
                          [0],
                          [0]])
        z = self.mu + delta

        #new_mu, new_Sigma = kalman_filter(mu, Sigma, u, z, delta_t)

    def kalman_filter_call(self, v, omega, delta_t):
        u = np.array([[v],
                      [omega]])

        delta = np.array([[0],
                          [0],
                          [0]])
        z = self.mu + delta

        self.kalman_filter_update(self.mu, self.Sigma, u, z, delta_t)


    def kalman_filter_update(self, mu, Sigma, u, z, delta_t):
        # slide 15
        A = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        # slide 15
        B = np.array([[delta_t * math.cos(mu[2]), 0],
                      [delta_t * math.cos(mu[2]), 0],
                      [0, delta_t]])

        # slide 16 # TODO
        R = np.array([[0.1, 0, 0],
                      [0, 0.2, 0],
                      [0, 0, 0.1]])

        # slide 26
        C = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        # slide 27 # TODO
        Q = np.array([[0.2, 0, 0],
                      [0, 0.1, 0],
                      [0, 0, 0.2]])

        I = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

        # prediction
        mu_estimate = np.matmul(A, mu) + np.matmul(B, u)
        Sigma_estimate = np.matmul(np.matmul(A, Sigma), A.transpose()) + R

        # correction
        K = np.matmul(np.matmul(Sigma_estimate, C.transpose()),
                      np.linalg.inv(np.matmul(np.matmul(C, Sigma_estimate), C.transpose()) + Q))

        mu = mu_estimate + np.matmul(K, (z - np.matmul(C, mu_estimate)))
        Sigma = np.matmul((I - np.matmul(K, C)), Sigma_estimate)

        self.mu = mu
        self.Sigma = Sigma

        self.positions.append((mu[0][0], mu[1][0]))


    def estimate_z(self, visible_landmarks, distances, bearing):
        """"
        1. new position (x, y) estimate
        a. calculate distances to visible landmarks
        b. per landmark take coordinates and distance for triangulation

        2. new orientation (theta) estimate
        a. calculate bearing of one landmark
        b. update orientation according

        3. create z from 1. and 2.
        """
        # 1. new position (x, y) estimate
        circles = []
        if len(visible_landmarks) == 3:
            for i, visible_landmark in enumerate(visible_landmarks):
                circles.append(visible_landmark.buffer(distances[i]))

            # find intersections of first and second circle
            intersections = circles[0].intersection(circles[1]) # two points
            # find intersection with third circle
            for point in intersections:
                intersection = point.intersection(circles[2])
                if not intersection.is_empty:
                    new_position = intersection




        else:
            # TODO
            pass

