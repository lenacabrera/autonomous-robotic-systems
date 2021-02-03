from particle import Particle
import numpy
import random

def pso(n_particles, n_iterations, benchmark_function):

    init_velocity = numpy.zeros(2)  # angle and distance  TODO

    # initialize all particles
    particles = []
    for i_particle in range(n_particles):
        x_rand = random.uniform(0, 10)  # TODO
        y_rand = random.uniform(0, 10)  # TODO
        init_position = numpy.array([x_rand, y_rand])
        particles.append(Particle(init_velocity, init_position))

    gbest = numpy.array([-1, -1])
    gbest_fitness = -1
    a = 0.9
    b = 2
    c = 2
    delta_t = 1

    # update
    for iteration in range(n_iterations):

        for particle in particles:
            particle.update(gbest, a, b, c, delta_t)
            particle.evaluate(benchmark_function)

            if particle.pbest_fitness > gbest_fitness:
                gbest_fitness = particle.pbest_fitness
                gbest = particle.pbest

        # decrease a
        decrease_amount = (0.9 - 0.4) / n_iterations
        a -= decrease_amount


if __name__ == '__main__':

    pso(5, 5, 'rosenbrock')
