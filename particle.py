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


    def update(self, gbest, a, b, c, delta_t, v_max, frame_range):
        # update velocity and position for one iteration
        r1 = random.uniform(0, 1)
        r2 = random.uniform(0, 1)
        v_new = a * self.v + b * r1 * (self.pbest - self.p) + c * r2 * (gbest - self.p)
        p_new = self.p + self.v * delta_t

        # what happen if particle move out of frame/search space? # TODO (add parameter)
        # 1. stick with old position
        if numpy.sqrt((v_new**2).sum(axis=0)) <= v_max:
            self.v = v_new
            print("change v ", v_new)
            if p_new[0] > frame_range[0] and p_new[1] > frame_range[0] and p_new[0] < frame_range[1] and p_new[1] < frame_range[1]:
                self.p = p_new
                print("change p ", p_new)
        # 2. move in opposite direction (x and/or y)

        # 3. new velocity with new random r1 and r2 (x and/or y)


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
