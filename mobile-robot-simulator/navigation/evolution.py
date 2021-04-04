from scipy.spatial import distance
from robot import Robot
from navigation.population import Population
from navigation import neural_network


def optimize(conf, robot, walls):


    population = Population(conf.n_individuals, conf.n_sensors, conf.hidden_dim)
    # copy_robot = Robot(x=c.x, y=c.y, radius=c.radius, n_sensors=c.n_sensors, max_sensor_reach=c.max_sensor_reach)
    # copy_robot = copy.deepcopy(robot)
    copy_robot = robot
    population.evaluate_population(conf, copy_robot, walls)
    print("Initial Fitness", population.fitness)
    termination_counter = 0
    n_generations = 0
    avg_fitnesses = []
    max_fitnesses = []
    diversity_measures = []

    while True:

        # GENERATION CYCLE
        selected = population.selection(conf)
        population.replacement(conf)
        population.crossover_and_mutation(conf)
        copy_robot = Robot(conf)
        # copy_robot = copy.deepcopy(robot)
        population.evaluate_population(conf, copy_robot, walls)

        diversity_measures.append(measure_diversity(population))

        # SIMULATION -> show best performing individual of current generation
        max_fitness = max(population.fitness)
        max_fitnesses.append(max_fitness)
        index_best_individuum = population.fitness.index(max_fitness)
        print("fitness_best", max(population.fitness))
        best_genotype = population.individuals[index_best_individuum]
        copy_robot = Robot(conf)
        # copy_robot = copy.deepcopy(robot)
        ann = neural_network.ANN(copy_robot.get_sensor_distance_values(walls), best_genotype, c.hidden_dim,
                                 copy_robot.max_sensor_reach)

        count = 0
        clock = pygame.time.Clock()
        for event in pygame.event.get():

            if count == 0:
                for steps in range(c.path_steps * 5 * n_generations):
                    v_left, v_right = ann.decode_genotype(copy_robot.get_sensor_distance_values(walls), best_genotype,
                                                          c.v_max)

                    copy_robot.set_new_position(c.delta_t, v_left, v_right)
                    copy_robot.robot_is_crossing_wall(walls)

                    copy_robot.update_sensors()

                    sensor_d = copy_robot.get_sensor_distance_values(walls)

                    # clear screen
                    screen.fill((255, 255, 255))

                    # drawGrid(screen)
                    drawPath(screen, copy_robot, c.path_color)

                    # draw scene
                    draw_walls(screen, walls, c.wall_thickness, c.wall_color)
                    draw_robot(screen, copy_robot, c.robot_color, sensor_d, font, draw_sensors=True)

                    text = "Generation " + str(n_generations)
                    textsurface = font.render(text, False, (0, 0, 0))
                    screen.blit(textsurface, (100, 50))

                    pygame.display.update()
                    clock.tick(60)

            else:
                break

            count += 1

        print("Current Fitness", population.fitness)

        # TERMINATION
        # evaluate fitness of current generation
        if n_generations == 0:
            old_avg_fitness = sum(population.fitness) / len(population.fitness)
        new_avg_fitness = sum(population.fitness) / len(population.fitness)
        if new_avg_fitness <= old_avg_fitness:
            # fitness stagnates or gets worse
            termination_counter += 1
        else:
            termination_counter = 0

        old_avg_fitness = new_avg_fitness
        avg_fitnesses.append(old_avg_fitness)
        if termination_counter >= c.termination_threshold or n_generations == c.max_n_generations:
            # terminate
            if termination_counter >= c.termination_threshold:
                print("Terminate - because fitness stagnates")
            if n_generations == c.max_n_generations:
                print("Terminate - because maximum number of generations reached")
            print("Average Fitness: ", old_avg_fitness)
            break

        # # evaluate fitness of current generation
        if n_generations == 0:
            old_best_fitness = max(population.fitness)
        new_best_fitness = max(population.fitness)
        print("\nold_best_fitness   new_best_fitness")
        print(old_best_fitness, new_best_fitness)
        old_best_fitness = new_best_fitness

        n_generations += 1
        print("Generation: ", n_generations)
        if n_generations == 10:

            plot_avg_and_max_fitness(n_generations, avg_fitnesses, max_fitnesses)
            plot_diversity(n_generations, diversity_measures)

            ##### Test best individual in different room
            test_room = "trapezoid"
            walls = room.init_walls_coordinates(c.env_width, c.env_height, c.wall_length, test_room)
            clock = pygame.time.Clock()
            for event in pygame.event.get():

                if count == 0:
                    for steps in range(c.path_steps):
                        print("steps ", steps)

                        v_left, v_right = ann.decode_genotype(copy_robot.get_sensor_distance_values(walls),
                                                              best_genotype,
                                                              c.v_max)
                        copy_robot.update_position(c.delta_t, v_left, v_right)
                        copy_robot.robot_is_crossing_wall(walls)

                        copy_robot.update_sensors()

                        sensor_d = copy_robot.get_sensor_distance_values(walls)

                        # clear screen
                        screen.fill((255, 255, 255))

                        drawPath(screen, copy_robot, c.path_color)

                        # draw scene
                        draw_walls(screen, walls, c.wall_thickness, c.wall_color)
                        draw_robot(screen, copy_robot, c.robot_color, sensor_d, font, draw_sensors=True)

                        text = "Generation " + str(n_generations)
                        textsurface = font.render(text, False, (0, 0, 0))
                        screen.blit(textsurface, (100, 50))

                        # update display
                        pygame.display.update()
                        clock.tick(60)

                else:
                    break

                count += 1


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



