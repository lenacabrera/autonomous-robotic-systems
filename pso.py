from particle import Particle
import numpy
import random
from pygame import *
import benchmark_functions

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.axes_grid1 import make_axes_locatable


def pso(n_particles, n_iterations, benchmark_function, a, b, c, r_max, delta_t, frame_range, random_init_v, v_max,
        oof_strategy):
    if random_init_v:
        # set velocity to random value
        init_velocity = numpy.array([random.uniform(-v_max, v_max), random.uniform(-v_max, v_max)])
    else:
        # set velocity to zero
        init_velocity = numpy.array([0, 0])

    # initialize all particles
    particles = []
    gbest = numpy.array([-1, -1])
    gbest_fitness = -1

    for i_particle in range(n_particles):
        x_rand = random.uniform(frame_range[0], frame_range[1])
        y_rand = random.uniform(frame_range[0], frame_range[1])
        init_position = numpy.array([x_rand, y_rand])
        print(init_position)
        particle = Particle(init_velocity, init_position)

        if benchmark_function == 'rosenbrock':
            p_fitness = benchmark_functions.rosenbrock(particle.p)

        if benchmark_function == 'rastrigin':
            p_fitness = benchmark_functions.rastrigin(particle.p)

        if p_fitness > gbest_fitness:
            gbest_fitness = p_fitness
            gbest = particle.p

        particles.append(particle)

    # init plot
    fig, ax = plt.subplots()
    y_coordinates, x_coordinates, heat = create_heatmap_data(benchmark_function, frame_range)
    div = make_axes_locatable(ax)
    cax = div.append_axes('right', '5%', '5%')
    # title = ax.text(0.5, 0.85, "", bbox={'facecolor': 'w', 'alpha': 0.5, 'pad': 5}, transform=ax.transAxes, ha="center")
    x_iterations = []
    y_iterations = []

    for iteration in range(n_iterations):

        for particle in particles:
            particle.update(gbest, a, b, c, r_max, delta_t, v_max, frame_range, oof_strategy)
            particle.evaluate(benchmark_function)

            if particle.pbest_fitness < gbest_fitness:
                gbest_fitness = particle.pbest_fitness
                gbest = particle.pbest

        print(iteration)

        # decrease a
        decrease_amount = (0.9 - 0.4) / n_iterations
        a -= decrease_amount

        x_pos, y_pos = create_scatter_data(particles)
        x_iterations.append(x_pos)
        y_iterations.append(y_pos)

        def animate(i):
            ax.clear()
            # ax.set_title('Iteration %s' % i)
            colormesh = plot_heatmap(y_coordinates, x_coordinates, heat, ax=ax)
            cax.cla()
            fig.colorbar(colormesh, cax=cax)
            x_pos = x_iterations[i]
            y_pos = y_iterations[i]
            print(i)
            return plot_scatter(x_pos, y_pos, ax=ax)  # , title

        ani = animation.FuncAnimation(fig, animate, interval=20, blit=True, frames=n_iterations,
                                      save_count=n_iterations)

    # ani.save("movie.mp4")

    return particles


def create_scatter_data(particles):
    x_coordinates = []
    y_coordinates = []

    for particle in particles:
        x_coordinates.append(particle.p[0])
        y_coordinates.append(particle.p[1])

    return x_coordinates, y_coordinates


def create_heatmap_data(benchmark_function, frame_range):
    heatmap_data = []

    x_coordinates = list(np.linspace(frame_range[0], frame_range[1], 200))
    y_coordinates = x_coordinates
    for y in y_coordinates:
        new_row = []
        for x in x_coordinates:
            if benchmark_function == "rosenbrock":
                new_row.append(benchmark_functions.rosenbrock([x, y]))
            if benchmark_function == "rastrigin":
                new_row.append(benchmark_functions.rastrigin([x, y]))
        heatmap_data.append(new_row)

    return y_coordinates, x_coordinates, numpy.array(heatmap_data)


def plot_scatter(x, y, ax=None, **kwargs):
    if ax is None:
        ax = plt.gca()
    artists = [ax.scatter(x, y, c='white') for x, y in zip(x, y)]
    return artists


def plot_heatmap(y_coordinates, x_coordinates, data, ax=None, **kwargs):
    if ax is None:
        ax = plt.gca()
    colormesh = ax.pcolormesh(y_coordinates, x_coordinates, data, shading='nearest')
    return colormesh


if __name__ == '__main__':
    pso(n_particles=20,  # 20
        n_iterations=130,  # 100
        benchmark_function='rastrigin',
        a=0.9,
        b=2,
        c=2,
        r_max=1,
        delta_t=1,
        frame_range=[-5, 5],  # rastrigin
        # frame_range=[-1, 1], # rosenbrock
        random_init_v=False,  # False=init with zero, True=random initi

        v_max=5,  # 1, 5 -> performs well
        # "out of screen strategy"
        # 0=old position,
        # 1=change direction,
        # 2=new random (r1, r2),
        # 3=only small step in new direction,
        # 4=old coordinate
        oof_strategy=4  # 4 is good
        )

    plt.show()
