from population import Population
import numpy as np
from neural_network import ANN

def evolutionary_algorithm(n_individuals, n_iterations, benchmark_function, n_best_percentage,
                           crossover_percentage, mutation_percentage, termination_threshold, num_sensors,
                           hidden_dim, sensor_distances, max_sensor_reach):

    population = Population(n_individuals, benchmark_function, num_sensors, hidden_dim)
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
            break

        n_generations += 1
        print("Generation: ", n_generations)

    return n_generations, fitnesses
