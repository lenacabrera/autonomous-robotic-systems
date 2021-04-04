from scipy.spatial import distance
import matplotlib.pyplot as plt


def measure_diversity(population):
    unique_individuals_tuple = list(set([tuple(g) for g in population.individuals]))
    unique_individuals = [list(t) for t in unique_individuals_tuple]

    distances = []
    for idx_individual, individual in enumerate(unique_individuals):
        for other_individual in unique_individuals[idx_individual:]:
            distances.append(distance.hamming(individual, other_individual))

    max_distance = max(distances)

    return max_distance

def plot_avg_and_max_fitness(n_generation, avg_fitnesses, max_fitnesses):
    generation = list(range(n_generation))
    fig, ax = plt.subplots()
    ax.plot(generation, avg_fitnesses, marker='.', label="Average")
    ax.plot(generation, max_fitnesses, marker='>', label="Max")
    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.title("Performance of Evolutionary Algorithm")
    plt.legend()


def plot_diversity(n_generation, diversity_measures):
    generation = list(range(n_generation))
    fig, ax = plt.subplots()
    ax.plot(generation, diversity_measures, marker='.')
    plt.xlabel("Generation")
    plt.ylabel("Diversity")
    plt.title("Diversity of Evolutionary Algorithm")
    # plt.legend()
    plt.show()