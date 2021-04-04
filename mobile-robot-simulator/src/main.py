import pygame
import sys
import copy
import imageio
import os
import robot
import environment
import animation
from localization.kalman_filter import KalmanFilter
from navigation.population import Population
from navigation.neural_network import ANN
from navigation import utils


class Configuration:

    def __init__(self):
        # environment
        self.env_width = 750                     # animation frame width
        self.env_height = self.env_width         # animation frame height
        self.wall_length = 600
        self.wall_thickness = 6
        self.wall_color = (196, 196, 196)
        self.room_shape = 'square'               # shape of environment: square, rectangle, rectangle_double, trapezoid, trapezoid_double

        # robot
        self.x = self.env_width / 2              # x coordinate of initial position of robot
        self.y = self.env_height / 2             # y coordinate of initial position of robot
        self.radius = self.env_width / 20        # radius of robot body
        self.position_initialization = "corner"  # robot's starting position: center, corner
        self.robot_color = (0, 153, 255)

        # sensors
        self.n_sensors = 12                      # number of sensors
        self.max_sensor_reach = 2 * self.radius  # maximum length of sensor beam
        self.visible_sensor_beams = True         # visualize sensor beams when True

        # motion and control
        self.o = 0.05                            # fixed increase/decrease of rotation
        self.v = 1.5                             # fixed increase/decrease of velocity
        self.v_max = 15                          # maximum velocity

        # animation
        self.delta_t = 1                         # time step
        self.uncertainty_update = 20             # visualize uncertainty every x time steps

        self.task = "localization"  # navigation, localization

        # localization with kalman filter (kf)
        # randomly drawing parameters from normal distribution (Gaussian) with
        self.kf_mean = 1                         # mean of normal distribution
        self.kf_std_Sigma = 0.01                 # uncertainty
        self.kf_std_R = 0.3                      # process noise
        self.kf_std_Q = 0.1                      # measurement noise (sensor)
        self.kf_std_z = 0.05                     # additional measurement noise
        self.kf_uncertainty_growth = 0.1         # degree to which uncertainty grows when no correction occurs
        self.save_steps = 200

        # navigation with evolutionary algorithm
        self.n_individuals = 30                  # population size
        self.max_n_generations = 50              # max. number of generations
        self.fitness_strategy = 0                # 0: only covered area, 1: 0 + reward collision-free navigation
        self.n_best_percentage = 0.8             # selection rate
        self.crossover_percentage = 0.2          # crossover rate: how many of selected to crossover
        self.mutation_percentage = 0.05          # mutation rate: likelihood to perform one mutation
        self.termination_threshold = 3           # fitness stagnation indicator
        self.dim_hidden = 4                      # memory size of ANN (neural controller)
        self.path_steps = 1                      # number of steps that robot makes in simulation (during evolution)
        self.test_steps = 500                    # number of steps that robot makes in simulation (after evolution)
        self.save_show_all_generations = False   # animate intermediate generations during evolution
        self.test_different_room = True          # test best evolved individual in a different environment (after evolution)
        self.test_room = "square"             # shape of test environment
        self.covered_area_color = (204, 229, 255)


def localization(conf, robot, walls, pygame, screen, timer_event):
    """ Performs localization with a Kalman Filter """

    landmarks = environment.get_landmarks(conf)
    kalman_filter = KalmanFilter(conf=conf, x=robot.x, y=robot.y, theta=0)
    uncertainty_increase = 0

    # for saving animation
    frame_files = ["./src/localization/gif/frame_" + str(f) + ".png" for f in range(conf.save_steps)]
    step = 0

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
                    kalman_filter.update_belief(robot, conf.delta_t, visible_landmarks, distances, bearings,
                                                uncertainty_increase)

                # check for landmarks
                visible_landmarks, distances, bearings = robot.check_landmarks_in_sight(landmarks)

                # clear screen
                screen.fill((255, 255, 255))

                # draw scene
                animation.draw_true_robot_trajectory(pygame, screen, robot)
                animation.draw_believed_robot_trajectory(pygame, screen, kalman_filter)
                animation.draw_uncertainty_ellipses(pygame, screen, kalman_filter, conf.uncertainty_update)
                animation.draw_walls(pygame, screen, walls, conf.wall_thickness, conf.wall_color)
                animation.draw_landmarks(pygame, screen, landmarks)
                animation.draw_robot(pygame, screen, font, robot, walls, visible_landmarks,
                                     visible_sensor_beams=conf.visible_sensor_beams)

                # store animation
                if step < conf.save_steps:
                    # save animation frame as PNG image
                    pygame.image.save(screen, frame_files[step])
                if step == conf.save_steps:
                    # create gif from PNG files
                    animation_frames = []
                    for frame_file in frame_files:
                        animation_frames.append(imageio.imread(frame_file))
                    imageio.mimsave('./src/localization/gif/localization.gif', animation_frames)
                    # remove the PNG files
                    for frame_file in frame_files:
                        os.remove(frame_file)
                step += 1

                # update screen
                pygame.display.update()


def navigation(conf, robot, walls, pygame, screen):
    """ Performs collision-free navigation with evolutionary algorithm with goal of maximizing the covered area """

    # create copy of initial robot
    robot_evolution = copy.deepcopy(robot)
    sensor_distances = robot.get_sensor_distance_values(walls)
    ann = ANN(conf, sensor_distances)

    # initialize population
    print("\nStart evolutionary cycle")
    population = Population(conf)
    population.evaluate_population(conf, robot_evolution, walls)
    old_avg_fitness = sum(population.fitness) / len(population.fitness)
    print("│ │  └─ Avg. Fitness: %s" % old_avg_fitness)
    print("│ │  └─ Max. Fitness: %s" % max(population.fitness))

    avg_fitnesses = []
    max_fitnesses = []
    diversity_measures = []
    termination_counter = 0
    generation_counter = 1

    # for saving animation
    frame_files = ["./src/navigation/gif/frame_" + str(f) + ".png" for f in range(conf.path_steps)]

    # run generation cycle
    while True:

        """ GENERATION (evolve population) """
        print("\n│ Generation %s" % generation_counter)

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
        new_avg_fitness = sum(population.fitness) / len(population.fitness)
        print("│ │  └─ Avg. Fitness: %s" % new_avg_fitness)
        print("│ │  └─ Max. Fitness: %s" % max_fitness)

        """ SIMULATION (with best performing individual from current generation) """
        if conf.save_show_all_generations:
            simulation_finished = False
            while pygame.event.get():
                # updating simulation frame
                if not simulation_finished:
                    print("│ └─ Navigation ...")
                    # copy initial robot for each simulation (with initial starting position)
                    robot_simulation = copy.deepcopy(robot)
                    for step in range(conf.path_steps):
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
                        screen.blit(font.render("Time step " + str(step), False, (0, 0, 0)), (200, 50))

                        # save animation frame as PNG image
                        pygame.image.save(screen, frame_files[step])

                        # update screen
                        pygame.display.update()

                simulation_finished = True

        """ EVALUATION CURRENT GENERATION """
        # evaluate fitness of current generation
        if new_avg_fitness <= old_avg_fitness:
            # fitness stagnates or gets worse
            termination_counter += 1
        else:
            # fitness (still) improves
            termination_counter = 0
        avg_fitnesses.append(new_avg_fitness)
        old_avg_fitness = new_avg_fitness

        """ TERMINATION EVOLUTION CYCLE """
        # check if termination threshold (stagnation indicator) or max. number of generations reached
        if termination_counter >= conf.termination_threshold or generation_counter == conf.max_n_generations:
            # terminate
            if termination_counter >= conf.termination_threshold:
                print("\nTerminate, because fitness stagnates")
            if generation_counter == conf.max_n_generations:
                print("\nTerminate, because maximum number of generations reached")
            print("Average fitness of %s. Generation: %s" % (generation_counter, old_avg_fitness))
            # plot current state of population (avg and max fitness, diversity)
            utils.plot_avg_and_max_fitness(generation_counter, avg_fitnesses, max_fitnesses)
            utils.plot_diversity(generation_counter, diversity_measures)

            if conf.save_show_all_generations:
                # create gif from PNG files
                animation_frames = []
                for frame_file in frame_files:
                    animation_frames.append(imageio.imread(frame_file))
                imageio.mimsave('./src/navigation/gif/generation_%s.gif' % generation_counter, animation_frames)

                # remove the PNG files
                for frame_file in frame_files:
                    os.remove(frame_file)

            """" PERFORMANCE OF FINAL (BEST) POPULATION """
            if conf.test_different_room:
                # test best individual in a different room
                print("\nChange environment... \nTest last generation ...")

                walls_test = environment.get_walls(conf, test_environment=conf.test_room)
                robot_test = copy.deepcopy(robot)

                # for saving animation
                test_frame_files = ["./src/navigation/gif/frame_" + str(f) + ".png" for f in range(conf.test_steps)]

                test_finished = False
                while pygame.event.get():
                    # updating simulation frame
                    if not test_finished:
                        for step in range(conf.test_steps):
                            v_left, v_right = ann.decode_genotype(robot_test.get_sensor_distance_values(walls_test),
                                                                  best_genotype, conf.v_max)
                            robot_test.update_position(conf.delta_t, v_left, v_right)
                            robot_test.collision_detection(walls_test)

                            # draw scene
                            screen.fill((255, 255, 255))  # clear screen
                            animation.draw_covered_area(pygame, screen, robot_test, conf.covered_area_color)
                            animation.draw_walls(pygame, screen, walls_test, conf.wall_thickness, conf.wall_color)
                            animation.draw_robot(pygame, screen, font, robot_test, walls_test, visible_landmarks=[],
                                                 visible_sensor_beams=conf.visible_sensor_beams)
                            screen.blit(font.render("Generation " + str(generation_counter), False, (0, 0, 0)), (100, 50))
                            screen.blit(font.render("Time step " + str(step), False, (0, 0, 0)), (200, 50))

                            # save animation frame as PNG image
                            pygame.image.save(screen, test_frame_files[step])

                            # update display
                            pygame.display.update()

                        test_finished = True

                # create gif from PNG files
                animation_frames = []
                for frame_file in test_frame_files:
                    animation_frames.append(imageio.imread(frame_file))
                imageio.mimsave('./src/navigation/gif/test_navigation.gif', animation_frames)

                # remove the PNG files
                for frame_file in test_frame_files:
                    os.remove(frame_file)

            break

        if conf.save_show_all_generations:
            # create gif from PNG files
            animation_frames = []
            for frame_file in frame_files:
                animation_frames.append(imageio.imread(frame_file))
            imageio.mimsave('./src/navigation/gif/generation_%s.gif' % generation_counter, animation_frames)

            # remove the PNG files
            for frame_file in frame_files:
                os.remove(frame_file)

        generation_counter += 1


if __name__ == '__main__':

    # load configuration
    conf = Configuration()

    # initialize pygame
    pygame.init()
    clock = pygame.time.Clock()
    timer_event = pygame.USEREVENT + 1
    pygame.time.set_timer(timer_event, 250)
    screen = pygame.display.set_mode([conf.env_width, conf.env_height])
    font = pygame.font.SysFont('Arial', 14)

    # setup environment
    walls = environment.get_walls(conf)

    if conf.task == "navigation":
        conf.delta_t *= 0.1
        # setup robot
        robot = robot.Differential_Drive_Robot(conf)
        navigation(conf, robot, walls, pygame, screen)
    if conf.task == "localization":
        conf.visible_sensor_beams = False
        # setup robot
        robot = robot.Velocity_Drive_Robot(conf)
        localization(conf, robot, walls, pygame, screen, timer_event)
