import pygame
import sys
import copy
import robot
import environment
import animation
from localization.localization import KalmanFilter
from navigation.population import Population
from navigation.neural_network import ANN
from navigation import utils


class Configuration:

    def __init__(self):
        # environment
        self.env_width = 750
        self.env_height = self.env_width
        self.room_shape = 'square'  # square, rectangle, rectangle_double, trapezoid, trapezoid_double
        self.wall_length = 600
        self.wall_thickness = 6
        self.wall_color = (196, 196, 196)

        # robot
        self.robot_type = "d"  # d -> differential, v - > velocity
        self.x = self.env_width / 2
        self.y = self.env_height / 2
        self.position_initialization = "center"  # robot's starting position: center, corner
        self.o = 0.05
        self.v = 0.5
        self.v_max = 15
        self.radius = self.env_width / 20
        self.robot_color = (0, 153, 255)
        self.n_sensors = 12
        self.max_sensor_reach = 2 * self.radius
        self.visible_sensor_beams = True

        # animation
        self.delta_t = 1  # time step
        self.uncertainty_update = 20

        """" Task
        # 0: Localization with Kalman Filter
        # 1: Navigation with Evolutionary Algorithm
        """
        self.task = 1

        # LOCALIZATION with kalman filter, randomly drawing from normal distribution (Gaussian) with
        self.kf_mean = 1
        self.kf_std_Sigma = 0.01
        self.kf_std_R = 0.7
        self.kf_std_Q = 0.3
        self.kf_std_z = 0.2
        self.kf_uncertainty_growth = 0.1

        # NAVIGATION with evolutionary algorithm, optimizing covered area
        self.n_individuals = 10  # 30
        self.max_n_generations = 10  # 100
        self.n_best_percentage = 0.8
        self.crossover_percentage = 0.2
        self.mutation_percentage = 0.05
        self.termination_threshold = 5
        self.dim_hidden = 4
        self.path_steps = 10  # 20
        self.test_different_room = True
        self.test_room = "trapezoid"
        self.covered_area_color = (204, 229, 255)


def localization(conf, robot, walls, pygame, screen, timer_event):
    """
    Performs localization with Kalman Filter: pose tracking with local localization (init position is known) and
    feature based map (landmark locations are known)
    """

    landmarks = environment.get_landmarks(conf)
    kalman_filter = KalmanFilter(conf=conf, x=robot.x, y=robot.y, theta=0)
    uncertainty_increase = 0

    # run simulation
    while True:
        for event in pygame.event.get():
            # for key / action
            if event.type == pygame.QUIT:
                # termination
                sys.exit()

            # steer robot with keystrokes
            robot.perform_motion(conf, pygame, robot)

            # update simulation frame
            if event.type == timer_event:
                robot.update_position(conf.delta_t)

                # check for collision with walls
                robot.collision_detection(walls)

                # check if robot is moving
                if (robot.v_wheel_l + robot.v_wheel_r) / 2 != 0:
                    if len(visible_landmarks) != 3:
                        uncertainty_increase += 1
                    else:
                        # no uncertainty increase if 3 landmarks are visible
                        uncertainty_increase = 0

                if (robot.v_wheel_l + robot.v_wheel_r) / 2 != 0:
                    # update kalman filter
                    kalman_filter.kalman_filter_update(robot, conf.delta_t, visible_landmarks, distances, bearings,
                                                       uncertainty_increase)

                # check for landmarks
                visible_landmarks, distances, bearings = robot.check_landmarks_in_sight(landmarks)

                # clear screen
                screen.fill((255, 255, 255))

                # draw scene
                if conf.task == 0:
                    animation.draw_true_robot_trajectory(pygame, screen, robot)
                    animation.draw_believed_robot_trajectory(pygame, screen, kalman_filter)
                    animation.draw_uncertainty_ellipses(pygame, screen, kalman_filter, conf.uncertainty_update)

                animation.draw_walls(pygame, screen, walls, conf.wall_thickness, conf.wall_color)
                animation.draw_landmarks(pygame, screen, landmarks)
                animation.draw_robot(pygame, screen, font, robot, walls, visible_landmarks,
                                     visible_sensor_beams=conf.visible_sensor_beams)

                # update screen
                pygame.display.update()


def navigation(conf, robot, walls, pygame, screen):
    """
    Collision-free navigation with evolutionary algorithm, goal: maximize covered area
    """

    # create copy of initial robot
    robot_evolution = copy.deepcopy(robot)
    sensor_distances = robot.get_sensor_distance_values(walls)
    ann = ANN(conf, sensor_distances)

    # initialize population
    population = Population(conf)
    population.evaluate_population(conf, robot_evolution, walls)

    avg_fitnesses = []
    max_fitnesses = []
    diversity_measures = []
    termination_counter = 0
    generation_counter = 0

    # run generation cycle
    while True:

        """ GENERATION (evolve population) """
        selected = population.selection(conf)
        population.replacement(selected)
        population.crossover_and_mutation(conf)
        population.evaluate_population(conf, robot_evolution, walls)

        # determine diversity of population
        diversity_measures.append(utils.measure_diversity(population))

        # determine best performing individual of current generation
        max_fitness = max(population.fitness)
        max_fitnesses.append(max_fitness)
        idx_best_individual = population.fitness.index(max_fitness)
        best_genotype = population.individuals[idx_best_individual]
        old_avg_fitness = sum(population.fitness) / len(population.fitness)

        """ SIMULATION (with best performing individual from current generation) """
        simulation_finished = False
        while pygame.event.get():
            # updating simulation frame
            if not simulation_finished:
                # copy initial robot for each simulation (with initial starting position)
                robot_simulation = copy.deepcopy(robot)
                for steps in range(conf.path_steps * 5 * generation_counter):
                    v_left, v_right = ann.decode_genotype(robot_simulation.get_sensor_distance_values(walls),
                                                          best_genotype, conf.v_max)
                    robot_simulation.update_position(conf.delta_t, v_left, v_right)
                    robot_simulation.collision_detection(walls)

                    # draw scene
                    screen.fill((255, 255, 255))  # clear screen
                    animation.draw_covered_area(pygame, screen, robot_simulation, conf.covered_area_color)
                    animation.draw_walls(pygame, screen, walls, conf.wall_thickness, conf.wall_color)
                    animation.draw_robot(pygame, screen, font, robot_simulation, walls, visible_landmarks=[],
                                         visible_sensor_beams=conf.visible_sensor_beams)
                    screen.blit(font.render("Generation " + str(generation_counter), False, (0, 0, 0)), (100, 50))

                    # update screen
                    pygame.display.update()

            simulation_finished = True

        """ EVALUATION CURRENT GENERATION """
        # evaluate fitness of current generation
        new_avg_fitness = sum(population.fitness) / len(population.fitness)
        if new_avg_fitness <= old_avg_fitness:
            # fitness stagnates or gets worse
            termination_counter += 1
        else:
            # fitness (still) improves
            termination_counter = 0
        old_avg_fitness = new_avg_fitness
        avg_fitnesses.append(old_avg_fitness)

        """ TERMINATION EVOLUTION CYCLE """
        generation_counter += 1
        print("%s. Generation with fitness: %s" % (generation_counter, max_fitness))
        # check if termination threshold (stagnation indicator) or max. number of generations reached
        if termination_counter >= conf.termination_threshold or generation_counter == conf.max_n_generations:
            # terminate
            if termination_counter >= conf.termination_threshold:
                print("\nTerminate, because fitness stagnates")
            if generation_counter == conf.max_n_generations:
                print("\nTerminate, because maximum number of generations reached")
            print("Average fitness of %s. Generation: %s" % (generation_counter, old_avg_fitness))
            break

        """" PERFORMANCE OF FINAL (BEST) POPULATION """
        if generation_counter == conf.max_n_generations:
            # plot current state of population (avg and max fitness, diversity)
            utils.plot_avg_and_max_fitness(generation_counter, avg_fitnesses, max_fitness)
            utils.plot_diversity(generation_counter, diversity_measures)

            # test best individual in a different room
            if conf.test_different_room:
                walls_test = environment.get_walls(conf, test_environment=conf.test_room)
                robot_test = copy.deepcopy(robot)

                while pygame.event.get():
                    # updating simulation frame
                    for steps in range(conf.path_steps):
                        v_left, v_right = ann.decode_genotype(robot_test.get_sensor_distance_values(walls_test),
                                                              best_genotype,
                                                              conf.v_max)
                        robot_test.update_position(conf.delta_t, v_left, v_right)
                        robot_test.collision_detection(walls_test)

                        # draw scene
                        screen.fill((255, 255, 255))  # clear screen
                        animation.draw_covered_area(pygame, screen, robot, conf.covered_area_color)
                        animation.draw_walls(pygame, screen, walls_test, conf.wall_thickness, conf.wall_color)
                        animation.draw_robot(pygame, screen, font, robot, walls_test, visible_landmarks=[],
                                             visible_sensor_beams=conf.visible_sensor_beams)
                        screen.blit(font.render("Generation " + str(generation_counter), False, (0, 0, 0)), (100, 50))

                        # update display
                        pygame.display.update()


if __name__ == '__main__':

    # load configuration
    conf = Configuration()

    # initialize pygame
    pygame.init()
    clock = pygame.time.Clock()
    timer_event = pygame.USEREVENT + 1  # initialize timer in order to provide constant timer-events
    pygame.time.set_timer(timer_event, 250)
    screen = pygame.display.set_mode([conf.env_width, conf.env_height])
    screen.fill((255, 255, 255))
    font = pygame.font.SysFont('Arial', 14)

    # setup environment
    walls = environment.get_walls(conf)

    # setup robot
    if conf.robot_type == "d":
        conf.delta_t *= 0.1
        robot = robot.Differential_Drive_Robot(conf)
    elif conf.robot_type == "v":
        robot = robot.Velocity_Drive_Robot(conf)

    if conf.task == 0:
        # localization with kalman filter
        localization(conf, robot, walls, pygame, screen, timer_event)
    if conf.task == 1:
        # navigation with evolutionary algorithm
        navigation(conf, robot, walls, pygame, screen)
