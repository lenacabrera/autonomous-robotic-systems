import copy
import random
import numpy as np
import fitness_function
import neural_network


class Population:

    def __init__(self, n_individuals, n_sensors, hidden_dim):

        self.individuals = self.build_population(n_individuals, n_sensors, hidden_dim)
        self.fitness = []

    def build_population(self, n_individuals, n_sensors, dim_hidden, bin_enc_len=7):
        """
        Initialize population according to the following encoding format:

        - binary representation of weights -> encoded as genotype
        - weight is between 0.01 and 0.001
        - structure of weight: prefix|suffix, between (0.0|100 and 0.0|010)
          - prefix always: 0.0
          - suffix: ___ between 100 and 10 (-> 010)
                        between 1100100 and 0001010
        - genotype only consists of suffix encodings (___) in binary numbers (length is always 7 digits -> len(1100100)

        EXAMPLE: if weight is 0.0056; then structure is 0.0|056, suffix is 056 = 56 = 0111000 <- one weight within genotype

        - entire genotype (length=504) -> bit0, bit2,..., bit503, where
            bit0   - bit447 -> weights input/hidden layer
            bit448 - bit503 -> weights hidden/output layer

            further:
            bit0   - bit6   -> weight FIRST sensor to FIRST hidden node
            bit7   - bit13  -> weight SECOND sensor to FIRST hidden node
            ...
            bit441 - bit447 -> weight FOURTH recurrent input to FOURTH hidden node
            ...
            bit447 - bit453 -> weight FIRST hidden node to FIRST output node
            ...
            bit497 - bit503 -> weight FOURTH hidden node to SECOND output node

        """

        dim_input = n_sensors + dim_hidden
        dim_output = 2
        n_weights_in_hid = dim_hidden * dim_input
        n_weights_hid_out = dim_output * dim_hidden
        n_weights = n_weights_in_hid + n_weights_hid_out

        population = []

        for i_individual in range(n_individuals):
            genotype = []

            for i_weight in range(n_weights):
                weight_suffix_int = random.randint(1, 100)
                weight_suffix_bin = [int(x) for x in list('{0:0b}'.format(weight_suffix_int))]
                weight = []

                # ensure that binary representations have the same length (length=7)
                preceeding_zeros = bin_enc_len - len(weight_suffix_bin)
                for preceeding_zero in range(preceeding_zeros):
                    weight.append(0)

                weight.extend(weight_suffix_bin)
                genotype.extend(weight)

            population.append(genotype)

        print(genotype)
        return population


    # def build_population(self, n_individuals, n_sensors, dim_hidden, bin_enc_len=7):
    #     """
    #     Initialize population according to the following encoding format:
    #
    #     - binary representation of weights -> encoded as genotype
    #     - weight is between 0.01 and 0.001
    #     - structure of weight: prefix|suffix, between (0.0|100 and 0.0|010)
    #       - prefix always: 0.0
    #       - suffix: ___ between 100 and 10 (-> 010)
    #                     between 1100100 and 0001010
    #     - genotype only consists of suffix encodings (___) in binary numbers (length is always 7 digits -> len(1100100)
    #
    #     EXAMPLE: if weight is 0.0056; then structure is 0.0|056, suffix is 056 = 56 = 0111000 <- one weight within genotype
    #
    #     - entire genotype (length=196) -> bit0, bit2,..., bit195, where
    #         bit0   - bit6   -> weight first sensor to LEFT wheel
    #         bit7   - bit13  -> weight second sensor to LEFT wheel
    #         ...
    #         bit91  - bit97  -> weight second recurrent input to LEFT wheel
    #
    #         bit98  - bit104 -> weight first sensor to RIGHT wheel
    #         bit105 - bit111 -> weight second sensor to RIGHT wheel
    #         ...
    #         bit189 - bit195 -> weight second recurrent input to RIGHT wheel
    #
    #     """
    #
    #     population = []
    #     n_weights = 2 * (n_sensors + 2)  # +2 for recurrent inputs in ANN
    #
    #     for i_individual in range(n_individuals):
    #
    #         genotype = []
    #
    #         for i_weight in range(n_weights):
    #
    #             weight_suffix_int = random.randint(1, 100)
    #             weight_suffix_bin = [int(x) for x in list('{0:0b}'.format(weight_suffix_int))]
    #             weight = []
    #
    #             # ensure that binary representations have the same length (length=7)
    #             preceeding_zeros = bin_enc_len - len(weight_suffix_bin)
    #             for preceeding_zero in range(preceeding_zeros):
    #                 weight.append(0)
    #
    #             weight.extend(weight_suffix_bin)
    #             genotype.extend(weight)
    #
    #         population.append(genotype)
    #         # print(genotype)
    #
    #     return population

    def evaluate_population(self, robot, walls, hidden_dim, delta_t, wall_length):
        fitness = []

        for individual in self.individuals:
            copy_robot = copy.deepcopy(robot)
            ann = neural_network.ANN(copy_robot.get_sensor_distance_values(walls), individual,
                                    hidden_dim, copy_robot.max_sensor_reach)

            for steps in range(20):
                v_left, v_right = ann.decode_genotype(copy_robot.get_sensor_distance_values(walls), individual)

                copy_robot.set_new_position(delta_t, v_left, v_right)
                copy_robot.robot_is_crossing_wall(walls)

                copy_robot.update_sensors()

            fitness.append(fitness_function.robot_fitness(robot, wall_length))

            # TODO put individual into ann
            # TODO robot object
            # position = self.genotype_to_weights(individual)
            # print(position)

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

    def crossover_and_mutation(self, crossover_percentage, mutation_percentage, n_best_percentage, bin_enc_len=7):

        def crossover(self):
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

                split_position = (random.randint(1, len(self.individuals[0]) / bin_enc_len - 1) * 7) - 1

                # do crossover
                new_ind_1 = ind_1[0:split_position]
                new_ind_1.extend(ind_2[split_position:])
                new_ind_2 = ind_2[0:split_position]
                new_ind_2.extend(ind_1[split_position:])

                # remove combination
                crossover_combinations.pop(idx_combination)

                # replace first individual (first occurring copy)
                for idx_ind, individual in enumerate(self.individuals):
                    if self.individuals[idx_ind] == ind_1:
                        self.individuals[idx_ind] = new_ind_1
                        break

                # replace second individual (first occurring copy)
                for idx_ind, individual in enumerate(self.individuals):
                    if self.individuals[idx_ind] == ind_2:
                        self.individuals[idx_ind] = new_ind_2
                        break

        def mutation(self):
            probability_range = list(range(0, 100))
            percentage_range = probability_range[0:int(100 * mutation_percentage)]

            # check if mutation should occur
            if random.randint(0, 100) in percentage_range:
                # select random individual
                idx_mutant = random.randint(0, len(self.individuals)-1)
                mutant = self.individuals[idx_mutant]

                # pick random bit to flip
                idx_bit = random.randint(0, len(mutant))
                bit = mutant[idx_bit]

                if bit == 1:
                    mutant[idx_bit] = 0
                    self.individuals[idx_mutant] = mutant
                else:
                    orig_indiv = mutant[:]
                    mutant[idx_bit] = 1

                    # check if mutation causes illegal weight
                    binary_start_idx = idx_bit - (idx_bit % bin_enc_len)
                    binary_end_idx = binary_start_idx + 7
                    binary_weight_suffix = mutant[binary_start_idx:binary_end_idx]
                    weight_suffix = 0
                    for bit in binary_weight_suffix:
                        weight_suffix = (weight_suffix << 1) | bit

                    if weight_suffix < 10 or binary_weight_suffix > 100:
                        # illegal weight -> do not flip bit, use random weight instead
                        weight_suffix_int = random.randint(1, 100)
                        weight_suffix_bin = [int(x) for x in list('{0:0b}'.format(weight_suffix_int))]
                        for w_idx, o_idx in enumerate(range(binary_start_idx, binary_end_idx)):
                            orig_indiv[o_idx] = weight_suffix_bin[w_idx]

                    self.individuals[idx_mutant] = orig_indiv

            # do crossover and mutation
            crossover()
            mutation()


    # def crossover_and_mutation(self, crossover_percentage, mutation_percentage, n_best_percentage):
    #
    #     # CROSSOVER
    #     n_crossover = int(int(len(self.individuals) * n_best_percentage) * crossover_percentage)
    #     unique_individuals_tuple = list(set([tuple(g) for g in self.individuals]))
    #     unique_individuals = [list(t) for t in unique_individuals_tuple]
    #
    #     # create all possible crossover combinations
    #     crossover_combinations = []
    #     for idx_individual, individual in enumerate(unique_individuals):
    #         individual = unique_individuals[idx_individual]
    #         for other_individual in unique_individuals[idx_individual + 1:]:
    #             crossover_combinations.append((individual, other_individual))
    #
    #     while n_crossover > len(crossover_combinations) / 2:
    #         n_crossover = n_crossover - 1
    #
    #     # randomly select n_crossover combinations
    #     for c_combination in range(n_crossover):
    #
    #         idx_combination = random.randint(0, len(crossover_combinations) - 1)
    #         combination_tuple = crossover_combinations[idx_combination]
    #         ind_1 = combination_tuple[0]
    #         ind_2 = combination_tuple[1]
    #
    #         # do crossover
    #         new_ind_1 = ind_1[0:int(len(ind_1) / 2)]
    #         new_ind_1.extend(ind_2[int(len(ind_2) / 2):])
    #         new_ind_2 = ind_2[0:int(len(ind_2) / 2)]
    #         new_ind_2.extend(ind_1[int(len(ind_1) / 2):])
    #
    #         # remove combination
    #         crossover_combinations.pop(idx_combination)
    #
    #         # replace first individual
    #         for idx_ind, individual in enumerate(self.individuals):
    #             if self.individuals[idx_ind] == ind_1:
    #                 self.individuals[idx_ind] = new_ind_1
    #                 break
    #
    #         # replace second individual
    #         for idx_ind, individual in enumerate(self.individuals):
    #             if self.individuals[idx_ind] == ind_2:
    #                 self.individuals[idx_ind] = new_ind_2
    #                 break
    #
    #     # MUTATION
    #     # check if mutation should occur
    #     whole_range = list(range(0, 100))
    #     percentage_range = whole_range[0:int(100*mutation_percentage)]
    #     if random.randint(0, 100) in percentage_range:
    #
    #         # select random individual
    #         idx_mutate = random.randint(0, len(self.individuals)-1)
    #         ind_to_mutate = self.individuals[idx_mutate]
    #
    #         # x = 0, y = 1
    #         coordinate_to_mutate = random.randint(0, 1)
    #
    #         if coordinate_to_mutate == 0:
    #             # mutate x coordinate
    #             random_digit = random.randint(0, 2)
    #
    #         else:
    #             # mutate y coordinate
    #             random_digit = random.randint(3, 5)
    #
    #         start = random_digit * 4
    #
    #         if ind_to_mutate[start] == 1:
    #             # change first digit
    #             ind_to_mutate[start] = 0
    #         else:
    #             # change second OR third digit
    #             if random.randint(0, 1) == 0:
    #                 # change second digit
    #                 if ind_to_mutate[start + 1] == 1:
    #                     ind_to_mutate[start + 1] = 0
    #                 else:
    #                     ind_to_mutate[start + 1] = 1
    #             else:
    #                 # change third digit
    #                 if ind_to_mutate[start + 2] == 1:
    #                     ind_to_mutate[start + 2] = 0
    #                 else:
    #                     ind_to_mutate[start + 2] = 1
    #
    #         self.individuals[idx_mutate] = ind_to_mutate
