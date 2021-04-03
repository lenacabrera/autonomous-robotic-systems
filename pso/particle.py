import random
import numpy
import benchmark_functions


class Particle:

    def __init__(self, velocity, position):
        self.v = velocity
        self.p = position
        self.pbest = position       # local best position (of this particular particle)
        self.pbest_fitness = 0      # local best fitness  (of this particular particle)
        self.pos_history = []       # particle's trajectory

    def update(self, gbest, a, b, c, r_max, delta_t, v_max, frame_range, oof_strategy):
        """ Updates velocity and position for one iteration """

        # random factors
        r1 = random.uniform(0, r_max)
        r2 = random.uniform(0, r_max)

        # velocity update
        v_new = a * self.v + b * r1 * (self.pbest - self.p) + c * r2 * (gbest - self.p)

        # measure the traveled distance / vector length
        distance = numpy.sqrt((v_new**2).sum(axis=0))
        if distance > v_max:
            # if greater than maximum velocity, keep old velocity
            v_new = self.v

        # position update
        p_new = self.p + v_new * delta_t

        # check if new position is outside of frame (oof)
        x_is_oof = p_new[0] < frame_range[0] or p_new[0] > frame_range[1]
        y_is_oof = p_new[1] < frame_range[0] or p_new[1] > frame_range[1]

        if x_is_oof or y_is_oof:
            x_counter = 2  # try strategy max. 2 times because replay can again lead to out of frame behavior...
            while x_is_oof and x_counter > 0:
                # x is out of frame
                if oof_strategy == 0:
                    # 0. keep old position and velocity
                    return
                elif oof_strategy == 1:
                    # 1. move in opposite direction
                    xv_new = - v_new[0]
                    v_new[0] = xv_new
                elif oof_strategy == 2:
                    # 2. new random r1 and r2
                    r1 = random.uniform(0, 1)
                    r2 = random.uniform(0, 1)
                    xv_new = a * self.v[0] + b * r1 * (self.pbest[0] - self.p[0]) + c * r2 * (gbest[0] - self.p[0])
                    v_new[0] = xv_new
                elif oof_strategy == 3:
                    # 3. make much smaller step in new direction
                    v_new[0] = 0.1 * v_new[0]
                elif oof_strategy == 4:
                    # 4. keep old x
                    v_new[0] = self.v[0]

                # position update
                p_new = self.p + v_new * delta_t
                # setup for another check of oof behavior
                x_is_oof = p_new[0] < frame_range[0] or p_new[0] > frame_range[1]
                x_counter -= 1

            if x_counter == 0:
                # keep old position and velocity
                return

            y_counter = 2  # try strategy max. 2 times because some can again lead to out of frame behavior...
            while y_is_oof and y_counter > 0:
                # y is out of frame
                if oof_strategy == 0:
                    # 1. keep old position
                    return
                elif oof_strategy == 1:
                    # 2. move in opposite direction
                    yv_new = - v_new[1]
                    v_new[1] = yv_new
                elif oof_strategy == 2:
                    # 3. new random r1 and r2
                    r1 = random.uniform(0, 1)
                    r2 = random.uniform(0, 1)
                    yv_new = a * self.v[1] + b * r1 * (self.pbest[1] - self.p[1]) + c * r2 * (gbest[1] - self.p[1])
                    v_new[1] = yv_new
                elif oof_strategy == 3:
                    # 4. make much smaller step in new direction
                    v_new[1] = 0.1 * v_new[1]
                elif oof_strategy == 4:
                    # 5. keep old y
                    v_new[1] = self.v[1]

                # position update
                p_new = self.p + v_new * delta_t
                # setup for another check of oof behavior
                y_is_oof = p_new[1] < frame_range[0] or p_new[1] > frame_range[1]
                y_counter -= 1

            if y_counter == 0:
                # keep old position and velocity
                return

        # update current velocity and position
        self.v = v_new
        self.p = p_new

    def evaluate(self, benchmark_function):
        """ Evaluates performance/fitness of a position """
        p_fitness = benchmark_functions.get_fitness(benchmark_function, self.p)
        pbest_fitness = benchmark_functions.get_fitness(benchmark_function, self.pbest)

        # check if fitness of current position is greater that best fitness found so far
        if pbest_fitness > p_fitness:
            # greater: update best
            self.pbest = self.p
            self.pbest_fitness = p_fitness
        else:
            # lower: keep old best
            self.pbest_fitness = pbest_fitness
