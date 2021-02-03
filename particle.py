# import pso
# update velocity and position for one iteration
import random
import benchmark_functions


class Particle:

    def __init__(self, velocity, position):
        self.v = velocity  # array with angle and distance
        self.p = position  # array with x and y
        self.pbest = position
        self.pbest_fitness = 0
        self.pos_history = []
        # self.performance = performance  # benchmark functions?

    def update(self, gbest, a, b, c, delta_t):
        # update velocity and position for one iteration
        r1 = random.uniform(0, 1)
        r2 = random.uniform(0, 1)
        v_new = a * self.v + b * r1 * (self.pbest - self.p) + c * r2 * (gbest - self.p)
        p_new = self.p + self.v * delta_t
        self.v = v_new
        self.p = p_new

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
