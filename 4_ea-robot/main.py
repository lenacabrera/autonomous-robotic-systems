import pygame
import sys
from robot import Robot
import math
import numpy as np
import matplotlib.pyplot as plt
from config import Configuration
import evolution
from neural_network import ANN
from population import Population


def plot_avg_fitness(n_generation, fitness):
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


def draw_walls(screen, walls, wall_thickness, wall_color):
    # left
    pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                     start_pos=walls['left'][0],
                     end_pos=walls['left'][1])

    # top
    pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                     start_pos=walls['top'][0],
                     end_pos=walls['top'][1])
    # right
    pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                     start_pos=walls['right'][0],
                     end_pos=walls['right'][1])

    # bottom
    pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                     start_pos=walls['bottom'][0],
                     end_pos=walls['bottom'][1])


def draw_robot(screen, robot, robot_color, distance_values, font, draw_sensors=False):
    # body of robot
    pygame.draw.circle(surface=screen, color=robot_color, center=(robot.x, robot.y), radius=robot.radius)
    # orientation of robot
    pygame.draw.line(surface=screen, color=(0, 0, 0), width=2,
                     start_pos=(robot.x, robot.y), end_pos=robot.orientation)
    # sensors
    if draw_sensors:
        for i_sensor, (x_sensor, y_sensor) in enumerate(robot.sensor_list):
            angle = (i_sensor + 1) * 360 / robot.num_sensors
            radians = math.atan2((y_sensor - robot.y), (x_sensor - robot.x))
            sensor_length = robot.radius + distance_values[i_sensor]

            sensor_start_x = robot.x + math.cos(radians) * robot.radius
            sensor_start_y = robot.y + math.sin(radians) * robot.radius
            sensor_end_x = robot.x + sensor_length * math.cos(radians)
            sensor_end_y = robot.y + sensor_length * math.sin(radians)

            pygame.draw.line(surface=screen, color=(255, 0, 0), width=1,
                             start_pos=(sensor_start_x, sensor_start_y), end_pos=(sensor_end_x, sensor_end_y))

            # display sensor values
            text = str(int(distance_values[i_sensor]))
            textsurface = font.render(text, False, (0, 0, 0))
            screen.blit(textsurface, (sensor_end_x - 7 + math.cos(angle * math.pi / 180) * robot.radius / 4,
                                      sensor_end_y - 7 + math.sin(angle * math.pi / 180) * robot.radius / 4))

    # display motor speed values
    text = str(int(robot.v_wheel_r))
    textsurface = font.render(text, False, (0, 0, 0))
    screen.blit(textsurface, (robot.x + np.cos(robot.line_angle + math.pi / 2) * robot.radius / 2,
                              robot.y + np.sin(robot.line_angle + math.pi / 2) * robot.radius / 2))

    text = str(int(robot.v_wheel_l))
    textsurface = font.render(text, False, (0, 0, 0))
    screen.blit(textsurface, (robot.x + np.cos(robot.line_angle + 3 * math.pi / 2) * robot.radius / 2,
                              robot.y + np.sin(robot.line_angle + 3 * math.pi / 2) * robot.radius / 2))


def init_walls_coordinates(env_width, env_height, wall_length):
    # wall frame distance
    dist_l_r = (env_width - wall_length) / 2  # distance to frame, left and right
    dist_t_b = (env_height - wall_length) / 2  # distance to frame, top and bottom

    left_start = (dist_l_r, dist_t_b)
    left_end = (dist_l_r, dist_t_b + wall_length)

    top_start = (dist_l_r, dist_t_b)
    top_end = (dist_l_r + wall_length, dist_t_b)

    right_start = (dist_l_r + wall_length, dist_t_b)
    right_end = (dist_l_r + wall_length, dist_t_b + wall_length)

    bottom_start = (dist_l_r, dist_t_b + wall_length)
    bottom_end = (dist_l_r + wall_length, dist_t_b + wall_length)

    walls = {
        'left': [left_start, left_end],
        'top': [top_start, top_end],
        'right': [right_start, right_end],
        'bottom': [bottom_start, bottom_end],
    }

    return walls


def initialize_pygame(c):
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode([c.env_width, c.env_height])
    screen.fill((255, 255, 255))  # background
    font = pygame.font.SysFont('Arial', 14)  # displayed numbers
    timer_event = pygame.USEREVENT + 1
    time = 250  # TODO: fine-tuning
    pygame.time.set_timer(timer_event, time)

    return screen, timer_event, font

def drawGrid(screen):
    blockSize = 40  # Set the size of the grid block
    for x in range(c.env_width):
        for y in range(c.env_height):
            # rect = pygame.Rect(x*blockSize, y*blockSize,
            #                    blockSize, blockSize)
            pygame.draw.rect(screen, (0, 0, 0), (x*blockSize, y*blockSize, blockSize, blockSize), 1)

def drawPath(screen, robot, robot_color):
    for position in robot.positions:
        pygame.draw.circle(surface=screen, color=robot_color,
                           center=(position[0], position[1]), radius=robot.radius)

if __name__ == '__main__':

    # load configuration parameters
    c = Configuration()

    # initializations
    screen, timer_event, font = initialize_pygame(c)
    robot = Robot(x=c.x, y=c.y, radius=c.radius, num_sensors=c.num_sensors, max_sensor_reach=c.max_sensor_reach)
    walls = init_walls_coordinates(c.env_width, c.env_height, c.wall_length)
    # grid = init_grid(50, c.env_width)

    # rectangle_walls = shapely.geometry.box(0, 0, c.wall_length, c.wall_length).area
    # print(rectangle_walls)

    screen.fill((255, 255, 255))
    # drawGrid(screen)
    empty = pygame.Color(255, 255, 255, 0)
    game_surf = pygame.surface.Surface((750, 750))

    # evolutionary algorithm
    while True:

        for event in pygame.event.get():
            # for key / action
            if event.type == pygame.QUIT:
                # termination
                sys.exit()

            pressed_keys = pygame.key.get_pressed()

            if c.steering == "keyboard":
                # manually drive robot

                # check it speed exceeds (positive) maximum velocity (forward movement)
                if (robot.v_wheel_l + robot.v_wheel_r) / 2 < c.v_max:
                    if pressed_keys[pygame.K_w]:
                        robot.v_wheel_l += c.v
                    if pressed_keys[pygame.K_o]:
                        robot.v_wheel_r += c.v
                    if pressed_keys[pygame.K_t]:
                        robot.v_wheel_l += c.v
                        robot.v_wheel_r += c.v
                    if pressed_keys[pygame.K_x]:
                        robot.v_wheel_l = 0
                        robot.v_wheel_r = 0

                # check it speed exceeds (negative) maximum velocity (backward movement)
                if (robot.v_wheel_l + robot.v_wheel_r) / 2 > - c.v_max:
                    if pressed_keys[pygame.K_s]:
                        robot.v_wheel_l -= c.v
                    if pressed_keys[pygame.K_l]:
                        robot.v_wheel_r -= c.v
                    if pressed_keys[pygame.K_g]:
                        robot.v_wheel_l -= c.v
                        robot.v_wheel_r -= c.v
                    if pressed_keys[pygame.K_x]:
                        robot.v_wheel_l = 0
                        robot.v_wheel_r = 0

            else:
                # autonomous driving
                pass

            # update screen by providing timer-event
            if event.type == timer_event:
                # update robot position
                robot.set_new_position(delta_t=0.1)

                # check for collision
                robot.robot_is_crossing_wall(walls)

                # sensor value update
                robot.update_sensors()
                sensor_d = robot.get_sensor_distance_values(walls)

                # clear screen
                #game_surf.fill(empty)
                screen.fill((255, 255, 255))

                drawGrid(screen)
                drawPath(screen, robot, c.robot_color)

                # draw scene
                draw_walls(screen, walls, c.wall_thickness, c.wall_color)
                draw_robot(screen, robot, c.robot_color, sensor_d, font, draw_sensors=True)

                # update display
                # screen.blit(game_surf, (0, 0))
                pygame.display.update()



            # #### TESTS
            # sensor_d = robot.get_sensor_distance_values(walls)
            # # sensor_d = [5, 10, 60, 20, 5, 12, 34, 65, 75, 55, 7, 25]
            # # genotype = [1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0]
            # # ann = ANN(sensor_d, genotype, c.hidden_dim, c.max_sensor_reach)
            # # velocities = ann.decode_genotype(sensor_d)
            # # print(velocities)
            #
            # population = Population(c.n_individuals, c.benchmark_function, c.num_sensors, c.hidden_dim)
            #
            # for genotype in population.individuals:
            #     ann = ANN(sensor_d, genotype, c.hidden_dim, c.max_sensor_reach)
            #     velocities = ann.decode_genotype(sensor_d)
            #     print(velocities)





        # n_generations, fitnesses = evolution.evolutionary_algorithm(n_individuals=c.n_individuals,
        #                                                             n_iterations=c.n_iterations,
        #                                                             benchmark_function=c.benchmark_function,
        #                                                             n_best_percentage=c.n_best_percentage,
        #                                                             crossover_percentage=c.crossover_percentage,
        #                                                             mutation_percentage=c.mutation_percentage,
        #                                                             termination_threshold=c.termination_threshold,
        #                                                             num_sensors=c.num_sensors,
        #                                                             hidden_dim=c.hidden_dim,
        #                                                             sensor_distances=sensor_d,
        #                                                             max_sensor_reach=c.max_sensor_reach
        #                                                             )

        # plot_avg_fitness(n_generations, fitnesses)
        # TODO plot max fitness
