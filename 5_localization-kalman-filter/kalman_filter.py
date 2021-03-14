import numpy as np
import math


def call_kalman_filter(x, y, Theta, v, omega, delta_t):
    # slide 9
    mu = np.array([[x],
                   [y],
                   [Theta]])

    # slide 9 # TODO
    Sigma = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

    # slide 15
    u = np.array([[v],
                  [omega]])

    # slide 26, TODO delta entspricht Q
    delta = np.array([[0],
                      [0],
                      [0]])
    z = mu + delta

    new_mu, new_Sigma = kalman_filter(mu, Sigma, u, z, delta_t)


def kalman_filter(mu, Sigma, u, z, delta_t):
    # slide 15
    A = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])

    # slide 15
    B = np.array([[delta_t * math.cos(mu[2]), 0],
                  [delta_t * math.cos(mu[2]), 0],
                  [0, delta_t]])

    # slide 16 # TODO
    R = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])

    # slide 26
    C = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])

    # slide 27 # TODO
    Q = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])

    I = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])

    # prediction
    mu_estimate = np.matmul(A, mu) + np.matmul(B, u)
    Sigma_estimate = np.matmul(np.matmul(A * Sigma) * A.transpose()) + R

    # correction
    K = np.matmul(np.matmul(Sigma_estimate, C.transpose()),
                  np.linalg.inv(np.matmul(np.matmul(C, Sigma_estimate), C.transpose()) + Q))

    mu = mu_estimate + K(z - np.matmul(C, mu_estimate))
    Sigma = np.matmul((I - np.matmul(K, C)), Sigma_estimate)

    return mu, Sigma
