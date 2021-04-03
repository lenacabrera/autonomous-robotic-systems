import numpy
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.axes_grid1 import make_axes_locatable
from particle import Particle
import benchmark_functions
from matplotlib import cm
import numpy as np
from celluloid import Camera


def optimize(n_particles, n_iterations, benchmark_function, a, b, c, r_max, delta_t, frame_range, random_init_v, v_max,
             oof_strategy):
    """ Performs particle swarm optimization """
    # build initial population
    particles, gbest, gbest_fitness = init_particles(n_particles, v_max, random_init_v, frame_range, benchmark_function)

    # initialize plot
    fig, ax = plt.subplots()
    div = make_axes_locatable(ax)
    cax = div.append_axes('right', '5%', '5%')
    coordinates, fitness_landscape = create_heatmap_data(benchmark_function, frame_range)
    x_iterations = []
    y_iterations = []

    for iteration in range(n_iterations):

        for particle in particles:
            # update particles' positions
            particle.update(gbest, a, b, c, r_max, delta_t, v_max, frame_range, oof_strategy)
            # evaluate particles' positions
            particle.evaluate(benchmark_function)

            # check if particle's fitness is greater than global best found so far TODO minimize instead maximize
            if particle.pbest_fitness < gbest_fitness:
                gbest_fitness = particle.pbest_fitness
                gbest = particle.pbest

        # decrease a (degree to which particle continues with previous velocity / "acting conservative")
        a -= (0.9 - 0.4) / n_iterations

        # retrieve particles' positions as data points for plot
        x_iterations, y_iterations = create_scatter_data(particles, x_iterations, y_iterations)

        def animate(i):
            ax.clear()
            cax.cla()
            colormesh = plot_heatmap(coordinates, fitness_landscape, ax=ax)
            fig.colorbar(colormesh, cax=cax)
            return plot_scatter(x_iterations[i], y_iterations[i], ax=ax)

        animation.FuncAnimation(fig, animate, blit=True)

    plt.show()


def init_particles(n_particles, v_max, random_init_v, frame_range, benchmark_function):
    """ Initializes a population of particles """
    particles = []
    gbest = numpy.array([-1, -1])
    gbest_fitness = -1

    if random_init_v:
        # set velocity to random value
        init_velocity = numpy.array([random.uniform(-v_max, v_max), random.uniform(-v_max, v_max)])
    else:
        # set velocity to zero
        init_velocity = numpy.array([0, 0])

    for i_particle in range(n_particles):
        # create particle
        x_rand = random.uniform(frame_range[0], frame_range[1])
        y_rand = random.uniform(frame_range[0], frame_range[1])
        init_position = numpy.array([x_rand, y_rand])
        particle = Particle(init_velocity, init_position)

        # evaluate particle
        p_fitness = benchmark_functions.get_fitness(benchmark_function, particle.p)

        # update global best position?
        if p_fitness > gbest_fitness:
            gbest = particle.p
            gbest_fitness = p_fitness

        particles.append(particle)

    return particles, gbest, gbest_fitness


def create_scatter_data(particles, x_iterations, y_iterations):
    """ Retrieve particles' positions for scatter plot """
    x_coordinates = []
    y_coordinates = []

    for particle in particles:
        x_coordinates.append(particle.p[0])
        y_coordinates.append(particle.p[1])

    x_iterations.append(x_coordinates)
    y_iterations.append(y_coordinates)

    return x_iterations, y_iterations


def create_heatmap_data(benchmark_function, frame_range):
    """ Plots fitness landscape of function """
    heatmap_data = []
    x_coordinates = list(numpy.linspace(frame_range[0], frame_range[1], 200))
    y_coordinates = x_coordinates

    for y in y_coordinates:
        new_row = []
        for x in x_coordinates:
            if benchmark_function == "rosenbrock":
                new_row.append(benchmark_functions.rosenbrock([x, y]))
            if benchmark_function == "rastrigin":
                new_row.append(benchmark_functions.rastrigin([x, y]))
        heatmap_data.append(new_row)

    coordinates = [x_coordinates, y_coordinates]
    fitness_landscape = numpy.array(heatmap_data)
    return coordinates, fitness_landscape


def plot_scatter(x, y, ax):
    """ Plot particles' positions """
    artists = [ax.scatter(x, y, c='white') for x, y in zip(x, y)]
    return artists


def plot_heatmap(coordinates, data, ax=None):
    """ Plot fitness landscape """
    if ax is None:
        ax = plt.gca()
    colormesh = ax.pcolormesh(coordinates[0], coordinates[1], data, shading='nearest')
    return colormesh
