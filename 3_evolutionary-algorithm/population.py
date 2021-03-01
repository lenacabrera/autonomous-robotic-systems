import random
import benchmark_functions

class Population:

    def __init__(self, n_individuals, frame_range, b_func):

        self.individuals = self.build_population(n_individuals, frame_range)
        self.fitness = []

    def build_population(self, n_particles, frame_range):
        # representation -> binary or real-valued?
        # encoding the position (value that minimizes the benchmark functions) -> 2 weights to optimize
        # -> 2 genes
        # 3 Ziffern: _ , _ _  -> x|y: 1000 , 1000 1000 | 1000 , 1000 1000
        # works only properly with frame_range of 9!
        population = []

        for i_particle in range(n_particles):
            genotype = []

            for k in range(2):

                random_1 = random.randint(0, 8)
                random_2 = random.randint(0, 9)
                random_3 = random.randint(0, 9)

                # first digit
                binary_1 = [int(x) for x in list('{0:0b}'.format(random_1))]
                while len(binary_1) < 4:
                    binary_1 = [0] + binary_1
                genotype.extend(binary_1)
                if binary_1[0] == 1:
                    for i in range(8):
                        # second + third digit
                        genotype.append(0)
                else:
                    # second digit
                    binary_2 = [int(x) for x in list('{0:0b}'.format(random_2))]
                    while len(binary_2) < 4:
                        binary_2 = [0] + binary_2
                    genotype.extend(binary_2)
                    # third digit
                    binary_3 = [int(x) for x in list('{0:0b}'.format(random_3))]
                    while len(binary_3) < 4:
                        binary_3 = [0] + binary_3
                    genotype.extend(binary_3)
            population.append(genotype)

        return population

    def toPhenotype(self, genotype):
        x = ""
        y = ""

        start = 0
        step = 4

        for i in range(int(len(genotype)/step)):

            out = 0
            for bit in genotype[start:start + step]:
                out = (out << 1) | bit

            if i < 3:
                x += str(out)
            else:
                y += str(out)
            if i == 0:
                x += "."

            if i == 3:
                y += "."

            start += step

        # print(genotype)
        # print(out)
        return float(x) - 4, float(y) - 4

    def evaluate_population(self, b_func):
        fitness = []
        for individual in self.individuals:
            position = self.toPhenotype(individual)
            # print(position)
            if b_func == 'rosenbrock':
                fitness.append(benchmark_functions.rosenbrock(position))
            if b_func == 'rastrigin':
                fitness.append(benchmark_functions.rosenbrock(position))
        self.fitness = fitness

    def selection(self, n_best_percentage):
        # truncated rank-based
        n_best = int(len(self.individuals) * n_best_percentage)
        zipped = zip(self.fitness, self.individuals)
        zipped_sorted = sorted(zipped, key=lambda x: x[0], reverse=False)
        return zipped_sorted[0:n_best]

    def replacement(self, selected):
        # generational replacement
        n_copy = int(len(self.individuals) / len(selected))
        difference = len(self.individuals) - (len(selected) * n_copy)

        new_population = []
        for genotype in selected:
            for i in range(n_copy):
                new_population.append(genotype[1])

        for genotype in selected:
            if difference > 0:
                new_population.append(genotype[1])
                difference -= 1
            else:
                break
        self.individuals = new_population

    def crossover_and_mutation(self, crossover_percentage, mutation_percentage, n_best_percentage):

        # CROSSOVER
        n_crossover = int(int(len(self.individuals) * n_best_percentage) * crossover_percentage)
        unique_individuals_tuple = list(set([tuple(g) for g in self.individuals]))
        unique_individuals = [list(t) for t in unique_individuals_tuple]

        # create all possible crossover combinations
        crossover_combinations = []
        for idx_individual, individual in enumerate(unique_individuals):
            individual = unique_individuals[idx_individual]
            for other_individual in unique_individuals[idx_individual + 1:]:
                crossover_combinations.append((individual, other_individual))

        while n_crossover > len(crossover_combinations) / 2:
            n_crossover = n_crossover - 1

        # randomly select n_crossover combinations
        for c_combination in range(n_crossover):

            idx_combination = random.randint(0, len(crossover_combinations) - 1)
            combination_tuple = crossover_combinations[idx_combination]
            ind_1 = combination_tuple[0]
            ind_2 = combination_tuple[1]

            # do crossover
            new_ind_1 = ind_1[0:int(len(ind_1) / 2)]
            new_ind_1.extend(ind_2[int(len(ind_2) / 2):])
            new_ind_2 = ind_2[0:int(len(ind_2) / 2)]
            new_ind_2.extend(ind_1[int(len(ind_1) / 2):])

            # remove combination
            crossover_combinations.pop(idx_combination)

            # replace first individual
            for idx_ind, individual in enumerate(self.individuals):
                if self.individuals[idx_ind] == ind_1:
                    self.individuals[idx_ind] = new_ind_1
                    break

            # replace second individual
            for idx_ind, individual in enumerate(self.individuals):
                if self.individuals[idx_ind] == ind_2:
                    self.individuals[idx_ind] = new_ind_2
                    break

        # MUTATION
        # check if mutation should occur
        whole_range = list(range(0, 100))
        percentage_range = whole_range[0:int(100*mutation_percentage)]
        if random.randint(0, 100) in percentage_range:

            # select random individual
            idx_mutate = random.randint(0, len(self.individuals)-1)
            ind_to_mutate = self.individuals[idx_mutate]

            # x = 0, y = 1
            coordinate_to_mutate = random.randint(0, 1)

            if coordinate_to_mutate == 0:
                # mutate x coordinate
                random_digit = random.randint(0, 2)

            else:
                # mutate y coordinate
                random_digit = random.randint(3, 5)

            start = random_digit * 4

            if ind_to_mutate[start] == 1:
                # change first digit
                ind_to_mutate[start] = 0
            else:
                # change second OR third digit
                if random.randint(0, 1) == 0:
                    # change second digit
                    if ind_to_mutate[start + 1] == 1:
                        ind_to_mutate[start + 1] = 0
                    else:
                        ind_to_mutate[start + 1] = 1
                else:
                    # change third digit
                    if ind_to_mutate[start + 2] == 1:
                        ind_to_mutate[start + 2] = 0
                    else:
                        ind_to_mutate[start + 2] = 1

            self.individuals[idx_mutate] = ind_to_mutate
