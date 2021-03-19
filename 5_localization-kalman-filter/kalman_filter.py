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
        delta = np.array([[0],
                          [0],
                          [0]])
        z = self.mu + delta

        # new_mu, new_Sigma = kalman_filter(mu, Sigma, u, z, delta_t)

    def kalman_filter_call(self, v, omega, delta_t, visible_landmarks, distances, bearings):
        u = np.array([[v],
                      [omega]])

        z = self.estimate_z(visible_landmarks, distances, bearings)

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
        # 1. new position (x, y) estimate
        # circles = []
        if len(visible_landmarks) == 3:
        #     for i, visible_landmark in enumerate(visible_landmarks):
        #         circles.append(visible_landmark.buffer(distances[i]))
        #     print('1. ', circles)
        #     # find intersections of first and second circle
        #     intersections = circles[0].boundary.intersection(circles[1].boundary)  # two points
        #     # find intersection with third circle
        #     print('2. ', intersections)
        #     for point in intersections:
        #         intersection = point.intersection(circles[2].boundary)
        #         print('3. ', intersection)
        #         if not intersection.is_empty:
        #             new_position = intersection
        #             # print(new_position)
        #
        #     # take average of estimates for different landmarks when calculating orientation from bearing
        #     bearing_avg = sum(bearings) / len(bearings)
        #     z = np.array([[new_position.x],
        #               [new_position.y],
        #               [bearing_avg]])

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
            # TODO
            delta = np.array([[0],
                              [0],
                              [0]])
            z = self.mu + delta

        return z
