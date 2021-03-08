import neural_network
import copy
import pygame
import sys
from robot import Robot
import math
import numpy as np
from config import Configuration
from population import Population
import room


def draw_walls(screen, walls, wall_thickness, wall_color):

    for left_wall in walls["left"]:
        pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                         start_pos=left_wall[0],
                         end_pos=left_wall[1])

    for top_wall in walls["top"]:
        pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                         start_pos=top_wall[0],
                         end_pos=top_wall[1])

    for right_wall in walls["right"]:
        pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                         start_pos=right_wall[0],
                         end_pos=right_wall[1])

    for bottom_wall in walls["bottom"]:
        pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                         start_pos=bottom_wall[0],
                         end_pos=bottom_wall[1])

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

def drawPath(screen, robot, path_color):
    for position in robot.positions:
        pygame.draw.circle(surface=screen, color=path_color,
                           center=(position[0], position[1]), radius=robot.radius)

def draw_generation_info(generation, env_width, avg_fitness):

    generation_str = "Generation " + str(generation)
    fitness_str = "Fitness: " + str(avg_fitness)
    generation_font = pygame.font.SysFont('Arial', 22)
    fitness_font = pygame.font.SysFont('Arial', 18)
    screen.blit(generation_font.render(generation_str, False, (0, 0, 0)), (20, 10))  # position in corner
    screen.blit(fitness_font.render(fitness_str, False, (0, 0, 0)), (20, 40))  # position in corner
    # screen.blit(textsurface, (env_width/2-50, 20))  # position in center

def initialize_pygame(c):
    pygame.init()
    clock = pygame.time.Clock()
    screen = pygame.display.set_mode([c.env_width, c.env_height])
    screen.fill((255, 255, 255))  # background
    font = pygame.font.SysFont('Arial', 14)  # displayed numbers
    timer_event = pygame.USEREVENT + 1
    time = 250  # TODO: fine-tuning
    pygame.time.set_timer(timer_event, time)
    # game_surf = pygame.surface.Surface((c.env_width, c.env_height))

    return screen, timer_event, font


if __name__ == '__main__':

    # load configuration parameters
    c = Configuration()

    # initializations
    screen, timer_event, font = initialize_pygame(c)
    robot = Robot(x=c.x, y=c.y, radius=c.radius, num_sensors=c.num_sensors, max_sensor_reach=c.max_sensor_reach)
    walls = room.init_walls_coordinates(c.env_width, c.env_height, c.wall_length, c.room_shape)

    population = Population(c.n_individuals, c.num_sensors, c.hidden_dim)
    population.evaluate_population(robot, walls, c.hidden_dim, c.delta_t, c.wall_length, c.path_steps, c.v_max, c.termination_threshold)
    termination_counter = 0
    n_generations = 0
    fitnesses = []

    while True:

        # GENERATION CYCLE
        selected = population.selection(c.n_best_percentage)
        population.replacement(selected)
        population.crossover_and_mutation(c.crossover_percentage, c.mutation_percentage, c.n_best_percentage)
        population.evaluate_population(robot, walls, c.hidden_dim, c.delta_t, c.wall_length, c.path_steps, c.v_max, c.termination_threshold)


        # SIMULATION -> show best performing individual of current generation
        index_best_individuum = population.fitness.index(max(population.fitness))
        best_genotype = population.individuals[index_best_individuum]
        copy_robot = copy.deepcopy(robot)
        ann = neural_network.ANN(copy_robot.get_sensor_distance_values(walls), best_genotype, c.hidden_dim,
                                 copy_robot.max_sensor_reach)

        for steps in range(c.path_steps):
            for event in pygame.event.get():
                # for key / action
                if event.type == pygame.QUIT:
                    # termination
                    sys.exit()

                v_left, v_right = ann.decode_genotype(copy_robot.get_sensor_distance_values(walls), best_genotype, c.v_max)

                if event.type == timer_event:
                    copy_robot.set_new_position(c.delta_t, v_left, v_right)
                    copy_robot.robot_is_crossing_wall(walls)

                    copy_robot.update_sensors()

                    sensor_d = copy_robot.get_sensor_distance_values(walls)

                    # clear screen
                    #game_surf.fill(empty)
                    screen.fill((255, 255, 255))

                    #drawGrid(screen)
                    drawPath(screen, copy_robot, c.path_color)

                    # draw scene
                    draw_walls(screen, walls, c.wall_thickness, c.wall_color)
                    draw_robot(screen, copy_robot, c.robot_color, sensor_d, font, draw_sensors=True)

                    # update display
                    # screen.blit(game_surf, (0, 0))
                    pygame.display.update()



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
        print("\nold_avg_fitness   new_avg_fitness")
        print(old_avg_fitness, new_avg_fitness)
        old_avg_fitness = new_avg_fitness
        fitnesses.append(old_avg_fitness)
        if termination_counter >= c.termination_threshold or n_generations == c.n_iterations:
            # terminate
            if termination_counter >= c.termination_threshold:
                print("Terminate - because fitness stagnates")
            if n_generations == c.n_iterations:
                print("Terminate - because maximum number of generations reached")
            print("Average Fitness: ", old_avg_fitness)
            break


        # print(population.fitness)
        # # evaluate fitness of current generation
        # if n_generations == 0:
        #     old_best_fitness = max(population.fitness)
        # new_best_fitness = max(population.fitness)
        # if new_best_fitness <= old_best_fitness:
        #     # fitness stagnates or gets worse
        #     termination_counter += 1
        # else:
        #     termination_counter = 0
        # print("\nold_best_fitness   new_best_fitness")
        # print(old_best_fitness, new_best_fitness)
        # old_avg_fitness = new_best_fitness
        # fitnesses.append(old_avg_fitness)
        # if termination_counter >= c.termination_threshold or n_generations == c.n_iterations:
        #     # terminate
        #     if termination_counter >= c.termination_threshold:
        #         print("Terminate - because fitness stagnates")
        #     if n_generations == c.n_iterations:
        #         print("Terminate - because maximum number of generations reached")
        #     print("Average Fitness: ", old_avg_fitness)
        #     break

        n_generations += 1
        draw_generation_info(n_generations, c.env_width, max(population.fitness))
        pygame.display.update()
        print("Generation: ", n_generations)
