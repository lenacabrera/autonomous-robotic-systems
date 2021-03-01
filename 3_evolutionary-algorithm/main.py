
from population import Population
import numpy
import random
import benchmark_functions
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.axes_grid1 import make_axes_locatable

# matplotlib.use("TkAgg")  # for use on MAC

# This code was jointly programmed by Kathrin Hartmann and Lena Cabrera


def evolutionary_algorithm(n_individuals, n_iterations, benchmark_function, frame_range, n_best_percentage,
                           crossover_percentage, mutation_percentage, termination_threshold):
    # init plot
    fig, ax = plt.subplots()
    y_coordinates, x_coordinates, heat = create_heatmap_data(benchmark_function, frame_range)
    div = make_axes_locatable(ax)
    cax = div.append_axes('right', '5%', '5%')
    x_iterations = []
    y_iterations = []

    population = Population(n_individuals, frame_range, benchmark_function)
    population.evaluate_population(benchmark_function)
    termination_counter = 0
    n_generations = 0
    fitnesses = []

    while True:

        # GENERATION CYCLE
        selected = population.selection(n_best_percentage)
        population.replacement(selected)
        population.crossover_and_mutation(crossover_percentage, mutation_percentage, n_best_percentage)
        population.evaluate_population(benchmark_function)

        # TERMINATION
        # evaluate fitness of current generation
        if n_generations == 0:
            old_avg_fitness = sum(population.fitness) / len(population.fitness)
        new_avg_fitness = sum(population.fitness) / len(population.fitness)
        if new_avg_fitness >= old_avg_fitness:
            # fitness stagnates or gets worse
            termination_counter += 1
        else:
            termination_counter = 0
        print("\nold_avg_fitness   new_avg_fitness")
        print(old_avg_fitness, new_avg_fitness)
        old_avg_fitness = new_avg_fitness
        fitnesses.append(old_avg_fitness)
        if termination_counter >= termination_threshold or n_generations == n_iterations:
            # terminate
            if termination_counter >= termination_threshold:
                print("Terminate - because fitness stagnates")
            if n_generations == n_iterations:
                print("Terminate - because maximum number of generations reached")
            print("Average Fitness: ", old_avg_fitness)
            for i in population.individuals:
                print(population.toPhenotype(i))
            break

        n_generations += 1
        print("Generation: ", n_generations)

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
        print("Iteration: ", i)
        return plot_scatter(x_pos, y_pos, i, ax=ax)

    animation.FuncAnimation(fig, animate, interval=1000, blit=True, frames=n_generations, save_count=n_generations)
    plot_standard_error(n_generations, fitnesses)


def create_scatter_data(population):
    x_coordinates = []
    y_coordinates = []
    positions = []
    for individual in population.individuals:
        position = population.toPhenotype(individual)
        positions.append(position)
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


def plot_scatter(x, y, i, ax=None):
    if ax is None:
        ax = plt.gca()
    artists = [ax.scatter(x, y, c='white') for x, y in zip(x, y)]
    ttl = ax.text(3.5, 3.7, i, color='white')
    artists.append(ttl)
    return artists


def plot_heatmap(y_coordinates, x_coordinates, data, ax=None):
    if ax is None:
        ax = plt.gca()
    colormesh = ax.pcolormesh(y_coordinates, x_coordinates, data, shading='nearest')
    return colormesh


def plot_standard_error(n_generation, fitness):
    # generation = np.array([1, 2, 3, 4, 5])
    # fitness = np.power(x, 2)  # Effectively y = x**2
    # e = np.array([1.5, 2.6, 3.7, 4.6, 5.5])
    generation = list(range(2, n_generation + 1))
    print(fitness)
    fitness.pop(0)
    fitness.pop(0)

    fig, ax = plt.subplots()
    ax.plot(generation, fitness, marker='.')
    plt.xlabel("Generation")
    plt.ylabel("Average Fitness")
    plt.title("Performance of Evolutionary Algorithm")

    plt.show()


if __name__ == '__main__':
    evolutionary_algorithm(n_individuals=109,
                           n_iterations=100,
                           benchmark_function='rastrigin',  # rastrigin, rosenbrock
                           frame_range=[-4, 4],
                           n_best_percentage=0.3,     # 0.2
                           crossover_percentage=0.6,  # 0.8, 0.6
                           mutation_percentage=0.1,  # 0.1, 0.05
                           termination_threshold=5    # 5
                           )

    plt.show()
