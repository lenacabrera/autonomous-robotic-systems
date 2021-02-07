# import pso
# update velocity and position for one iteration
import random

import numpy
from pygame import *

import benchmark_functions


class Particle:

    def __init__(self, velocity, position):
        self.v = velocity  # array with angle and distance
        self.p = position  # array with x and y
        self.pbest = position
        self.pbest_fitness = 0
        self.pos_history = []
        # self.performance = performance  # benchmark functions?

        self.size = 5
        self.colour = (0, 0, 255)
        self.thickness = 5

    def display(self, screen, frame_range):
        draw.circle(screen, self.colour, (self.p[0] + frame_range[1], self.p[1] + frame_range[1]), self.size, self.thickness)

    def update(self, gbest, a, b, c, r_max, delta_t, v_max, frame_range, oof_strategy):
        # update velocity and position for one iteration
        r1 = random.uniform(0, r_max)
        r2 = random.uniform(0, r_max)
        v_new = a * self.v + b * r1 * (self.pbest - self.p) + c * r2 * (gbest - self.p)

        distance = numpy.sqrt((v_new**2).sum(axis=0))
        if distance > v_max:
            # v_new == v_max
            # keep old velocity
            v_new = self.v
            print("distance greater than v_max")

        p_new = self.p + v_new * delta_t
        # p_new = self.p + self.v * delta_t

        # check if new position is outside of frame
        x_is_oof = p_new[0] < frame_range[0] or p_new[0] > frame_range[1]
        y_is_oof = p_new[1] < frame_range[0] or p_new[1] > frame_range[1]

        if x_is_oof or y_is_oof:

            x_counter = 2  # try strategy max. 2 times because some can again lead to out of frame behavior...
            while x_is_oof and x_counter > 0:
                # x is out of frame
                if oof_strategy == 0:
                    # keep old position and velocity
                    return
                elif oof_strategy == 1:
                    # move in opposite direction
                    xv_new = - v_new[0]
                    v_new[0] = xv_new
                elif oof_strategy == 2:
                    # new random r1 and r2
                    r1 = random.uniform(0, 1)
                    r2 = random.uniform(0, 1)
                    xv_new = a * self.v[0] + b * r1 * (self.pbest[0] - self.p[0]) + c * r2 * (gbest[0] - self.p[0])
                    v_new[0] = xv_new
                elif oof_strategy == 3:
                    # make much smaller step in new direction
                    v_new[0] = 0.1 * v_new[0]
                elif oof_strategy == 4:
                    # keep old x
                    v_new[0] = self.v[0]

                p_new = self.p + v_new * delta_t
                x_is_oof = p_new[0] < frame_range[0] or p_new[0] > frame_range[1]
                x_counter -= 1

            if x_counter == 0:
                # keep old position and velocity
                return

            y_counter = 2  # try strategy max. 2 times because some can again lead to out of frame behavior...
            while y_is_oof and y_counter > 0:
                # y is out of frame
                if oof_strategy == 0:
                    # keep old position
                    return
                elif oof_strategy == 1:
                    # move in opposite direction
                    yv_new = - v_new[1]
                    v_new[1] = yv_new
                elif oof_strategy == 2:
                    # new random r1 and r2
                    r1 = random.uniform(0, 1)
                    r2 = random.uniform(0, 1)
                    yv_new = a * self.v[1] + b * r1 * (self.pbest[1] - self.p[1]) + c * r2 * (gbest[1] - self.p[1])
                    v_new[1] = yv_new
                elif oof_strategy == 3:
                    # make much smaller step in new direction
                    v_new[1] = 0.1 * v_new[1]
                elif oof_strategy == 4:
                    # keep old y
                    v_new[1] = self.v[1]

                p_new = self.p + v_new * delta_t
                y_is_oof = p_new[1] < frame_range[0] or p_new[1] > frame_range[1]
                y_counter -= 1

            if y_counter == 0:
                # keep old position and velocity
                return

        # update current velocity and position
        self.v = v_new
        self.p = p_new
        print(self.p)

    def evaluate(self, b_func):
        # evaluate performance/fitness of a position
        if b_func == 'rosenbrock':
            p_fitness = benchmark_functions.rosenbrock(self.p)
            pbest_fitness = benchmark_functions.rosenbrock(self.pbest)
        if b_func == 'rastrigin':
            p_fitness = benchmark_functions.rastrigin(self.p)
            pbest_fitness = benchmark_functions.rastrigin(self.pbest)

        if pbest_fitness < p_fitness:
            self.pbest = self.p
            self.pbest_fitness = p_fitness
        else:
            self.pbest_fitness = pbest_fitness
