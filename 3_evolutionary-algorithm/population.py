import random
import benchmark_functions

class Population:

    def __init__(self, n_individuals, frame_range, b_func):

        self.individuals = self.build_population(n_individuals, frame_range)
        self.fitness = []


    # def build_population(self, n_particles, frame_range):
    #     # representation -> binary or real-valued?
    #     # encoding the position (value that minimizes the benchmark functions) -> 2 weights to optimize
    #     # -> 2 genes
    #     # TODO: check again (too many 4s)
    #     # 3 Ziffern: _ , _ _  -> x|y: 1000 , 1000 1000 | 1000 , 1000 1000
    #     # works only properly with frame_range of 9!
    #     population = []
    #
    #     for i_particle in range(n_particles):
    #         genotype = []
    #
    #         for k in range(2):
    #
    #             bit_1 = random.getrandbits(1)
    #             genotype.append(bit_1)
    #             if bit_1 == 1:
    #                 for i in range(11):
    #                     genotype.append(0)
    #             else:
    #                 genotype.append(random.getrandbits(1))
    #                 genotype.append(random.getrandbits(1))
    #                 genotype.append(random.getrandbits(1))
    #
    #                 bit_2 = random.getrandbits(1)
    #                 genotype.append(bit_2)
    #
    #                 if bit_2 == 1:
    #                     for i in range(3):
    #                         genotype.append(0)
    #                 else:
    #                     genotype.append(random.getrandbits(1))
    #                     genotype.append(random.getrandbits(1))
    #                     genotype.append(random.getrandbits(1))
    #
    #                 bit_3 = random.getrandbits(1)
    #                 genotype.append(bit_3)
    #
    #                 if bit_3 == 1:
    #                     for i in range(3):
    #                         genotype.append(0)
    #                 else:
    #                     genotype.append(random.getrandbits(1))
    #                     genotype.append(random.getrandbits(1))
    #                     genotype.append(random.getrandbits(1))
    #
    #         population.append(genotype)
    #
    #     return population

    def build_population(self, n_particles, frame_range):
        # representation -> binary or real-valued?
        # encoding the position (value that minimizes the benchmark functions) -> 2 weights to optimize
        # -> 2 genes
        # TODO: check again (too many 4s)
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

            # print(genotype)
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
        # print(zipped_sorted, "zipped sorted")
        return zipped_sorted[0:n_best]


    def replacement(self, selected):
        # generational replacement
        # TODO: only for populations which are multiples of percent of n_best
        n_copy = int(len(self.individuals) / len(selected))
        new_population = []
        for genotype in selected:
            for i in range(n_copy):
                new_population.append(genotype[1])
        self.individuals = new_population

        # print(self.individuals)

    def crossover_and_mutation(self):
        # crossover: first and last one
        ind_1 = self.individuals[0]
        ind_2 = self.individuals[-1]

        new_ind_1 = ind_1[0:int(len(ind_1) / 2)]
        new_ind_1.extend(ind_2[int(len(ind_2) / 2):])
        new_ind_2 = ind_2[0:int(len(ind_2) / 2)]
        new_ind_2.extend(ind_1[int(len(ind_1) / 2):])

        self.individuals[0] = new_ind_1
        self.individuals[-1] = new_ind_2

        # mutation
        ind_to_mutate = self.individuals[1]
        random_digit = random.randint(0,2)

        start = random_digit * 4

        if ind_to_mutate[start] == 1:
            ind_to_mutate[start] = 0
        elif ind_to_mutate[start + 1] == 1:
            ind_to_mutate[start + 1] = 0
        elif ind_to_mutate[start + 1] == 0:
            ind_to_mutate[start + 1] = 1

        self.individuals[1] = ind_to_mutate

