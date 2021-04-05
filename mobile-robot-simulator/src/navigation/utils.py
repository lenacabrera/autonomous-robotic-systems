from scipy.spatial import distance
import matplotlib.pyplot as plt
import os


def measure_diversity(population):
    """ Measures the (hamming) distance between an individual with all other individuals """

    unique_individuals_tuple = list(set([tuple(g) for g in population.individuals]))
    unique_individuals = [list(t) for t in unique_individuals_tuple]

    distances = []
    for idx_individual, individual in enumerate(unique_individuals):
        for other_individual in unique_individuals[idx_individual:]:
            distances.append(distance.hamming(individual, other_individual))

    max_distance = max(distances)

    return max_distance


def plot_avg_and_max_fitness(n_generations, avg_fitnesses, max_fitnesses):
    """ Plots average and maximum fitness of all generations """

    generations = list(range(n_generations))
    fig, ax = plt.subplots()
    ax.plot(generations, avg_fitnesses, marker='.', label="Average")
    ax.plot(generations, max_fitnesses, marker='>', label="Max")
    ax.set_xticklabels([g + 1 for g in generations])
    plt.xticks(generations)
    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.title("Performance of Evolutionary Algorithm")
    plt.legend()
    path_prefix = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(path_prefix + '/plots/fitness.png')


def plot_diversity(n_generations, diversity_measures):
    """ Plots diversity of all generations"""

    generations = list(range(n_generations))
    fig, ax = plt.subplots()
    ax.plot(generations, diversity_measures, marker='.')
    ax.set_xticklabels([g + 1 for g in generations])
    plt.xticks(generations)
    plt.xlabel("Generation")
    plt.ylabel("Diversity")
    plt.title("Diversity of Evolutionary Algorithm")
    path_prefix = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(path_prefix + '/plots/diversity.png')
