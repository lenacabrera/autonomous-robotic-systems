import matplotlib

from particle import Particle
from population import Population
import numpy
import random
import benchmark_functions
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.axes_grid1 import make_axes_locatable
matplotlib.use("TkAgg")
# This code was jointly programmed by Kathrin Hartmann and Lena Cabrera


def evolutionary_algorithm(n_particles, n_iterations, benchmark_function, frame_range, n_best_percentage, termination_threshold):

    population = Population(n_particles, frame_range, benchmark_function)

    termination_counter = 0

    # init plot
    fig, ax = plt.subplots()
    y_coordinates, x_coordinates, heat = create_heatmap_data(benchmark_function, frame_range)
    div = make_axes_locatable(ax)
    cax = div.append_axes('right', '5%', '5%')
    x_iterations = []
    y_iterations = []

    for iteration in range(n_iterations):
        population.evaluate_population(benchmark_function)
        selected = population.selection(n_best_percentage)
        population.replacement(selected)
        population.crossover_and_mutation()

        print(iteration)

        x_pos, y_pos = create_scatter_data(population)
        x_iterations.append(x_pos)
        y_iterations.append(y_pos)

        def animate(i):
            ax.clear()
            colormesh = plot_heatmap(y_coordinates, x_coordinates, heat, ax=ax)
            cax.cla()
            fig.colorbar(colormesh, cax=cax)
            x_pos = x_iterations[i]
            y_pos = y_iterations[i]
            print(i)
            return plot_scatter(x_pos, y_pos, ax=ax)

        animation.FuncAnimation(fig, animate, interval=20, blit=True, frames=n_iterations, save_count=n_iterations)


def create_scatter_data(population):
    x_coordinates = []
    y_coordinates = []

    for individual in population.individuals:
        position = population.toPhenotype(individual)
        x_coordinates.append(position[0])
        y_coordinates.append(position[1])

    return x_coordinates, y_coordinates


def create_heatmap_data(benchmark_function, frame_range):
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

    return y_coordinates, x_coordinates, numpy.array(heatmap_data)


def plot_scatter(x, y, ax=None):
    if ax is None:
        ax = plt.gca()
    artists = [ax.scatter(x, y, c='white') for x, y in zip(x, y)]
    return artists


def plot_heatmap(y_coordinates, x_coordinates, data, ax=None):
    if ax is None:
        ax = plt.gca()
    colormesh = ax.pcolormesh(y_coordinates, x_coordinates, data, shading='nearest')
    return colormesh


#def check_termination():
    # TODO
    # 1. max fitness or
    # 2. good enough or
    # 3. no improvement




if __name__ == '__main__':
    evolutionary_algorithm(n_particles=5,
        n_iterations=30,
        benchmark_function='rastrigin',  # rastrigin, rosenbrock
        frame_range=[-4, 4],
        n_best_percentage=0.2,
        termination_threshold=5
        )

    plt.show()
