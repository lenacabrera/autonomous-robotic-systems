import math
import numpy as np


def draw_walls(pygame, screen, walls, wall_thickness, wall_color):
    """ Draws walls of the environment as lines """
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


def draw_landmarks(pygame, screen, landmarks):
    """ Draws landmarks as dots in the environment """
    for landmark in landmarks:
        pygame.draw.circle(surface=screen, color=(196, 196, 196), center=(landmark.x, landmark.y), radius=5)


def draw_robot(pygame, screen, font, robot, walls, visible_landmarks, visible_sensor_beams=False):
    """ Draws the robot as a circle with a line indicating its orientation as well as lines depicting its sensor beams
    and line of vision for visible landmarks. """

    # body of robot
    pygame.draw.circle(surface=screen, color=robot.color, center=(robot.x, robot.y), radius=robot.radius, width=2)

    # orientation of robot
    pygame.draw.line(surface=screen, color=robot.color, width=2,
                     start_pos=(robot.x, robot.y), end_pos=robot.orientation)

    # motor speed values
    screen.blit(font.render(str(int(robot.v_wheel_l)), False, robot.color),
                (robot.x + np.cos(robot.theta + 3 * math.pi / 2) * robot.radius / 2,
                 robot.y + np.sin(robot.theta + 3 * math.pi / 2) * robot.radius / 2))

    screen.blit(font.render(str(int(robot.v_wheel_r)), False, robot.color),
                (robot.x + np.cos(robot.theta + math.pi / 2) * robot.radius / 2,
                 robot.y + np.sin(robot.theta + math.pi / 2) * robot.radius / 2))

    # sensors
    if visible_sensor_beams:
        sensor_distances = robot.get_sensor_distance_values(walls)

        for i_sensor, (x_sensor, y_sensor) in enumerate(robot.sensor_list):

            # sensor beam
            radians = math.atan2((y_sensor - robot.y), (x_sensor - robot.x))
            sensor_length = robot.radius + sensor_distances[i_sensor]

            sensor_start_x = robot.x + math.cos(radians) * robot.radius
            sensor_start_y = robot.y + math.sin(radians) * robot.radius
            sensor_end_x = robot.x + sensor_length * math.cos(radians)
            sensor_end_y = robot.y + sensor_length * math.sin(radians)

            pygame.draw.line(surface=screen, color=(204, 0, 102), width=1,
                             start_pos=(sensor_start_x, sensor_start_y), end_pos=(sensor_end_x, sensor_end_y))

            # sensor distance
            angle = (i_sensor + 1) * 360 / robot.n_sensors
            screen.blit(font.render(str(int(sensor_distances[i_sensor])), False, (204, 0, 102)),
                        (sensor_end_x - 7 + math.cos(angle * math.pi / 180) * robot.radius / 4,
                         sensor_end_y - 7 + math.sin(angle * math.pi / 180) * robot.radius / 4))

    # visible landmarks
    for visible_landmark in visible_landmarks:
        draw_dashed_line(pygame, screen, color=(0, 158, 0),
                         start_pos=(robot.x, robot.y), end_pos=(visible_landmark.x, visible_landmark.y))


def draw_true_robot_trajectory(pygame, screen, robot):
    """ Draws a line for the true trajectory of the robot (navigation) """
    robot_positions = robot.trajectory_positions
    for start in range(len(robot_positions) - 1):
        pygame.draw.line(surface=screen, color=(30, 144, 255), width=2,
                         start_pos=(robot_positions[start][0], robot_positions[start][1]),
                         end_pos=(robot_positions[start + 1][0], robot_positions[start + 1][1]))


def draw_believed_robot_trajectory(pygame, screen, kalman_filter):
    """ Draws a line for the believed trajectory of the robot """
    kalman_filter_positions = kalman_filter.positions
    for start in range(len(kalman_filter_positions) - 1):
        pygame.draw.line(surface=screen, color=(255, 127, 80), width=2,
                         start_pos=(kalman_filter_positions[start][0], kalman_filter_positions[start][1]),
                         end_pos=(kalman_filter_positions[start + 1][0], kalman_filter_positions[start + 1][1]))


def draw_uncertainty_ellipses(pygame, screen, kalman_filter, time_step):
    """ Visualizes the uncertainty of robot belief in form of ellipses (planar Gaussian) every x time steps"""
    for i, entry in enumerate(kalman_filter.uncertainty_history):
        if i % time_step == 0:
            pygame.draw.ellipse(surface=screen, color=(0, 0, 0), rect=(entry[0], entry[1], entry[2], entry[3]), width=1)


def draw_covered_area(pygame, screen, robot, covered_area_color):
    """ Colors the area that robot has already visited """
    for position in robot.trajectory_positions:
        pygame.draw.circle(surface=screen, color=covered_area_color,
                           center=(position[0], position[1]), radius=robot.radius)


def draw_generation_info(pygame, screen, generation, avg_fitness):
    """ Displays text information for animation """
    generation_str = "Generation " + str(generation)
    fitness_str = "Fitness: " + str(avg_fitness)
    generation_font = pygame.font.SysFont('Arial', 22)
    fitness_font = pygame.font.SysFont('Arial', 18)
    screen.blit(generation_font.render(generation_str, False, (0, 0, 0)), (20, 10))  # position in corner
    screen.blit(fitness_font.render(fitness_str, False, (0, 0, 0)), (20, 40))  # position in corner


def draw_dashed_line(pygame, surf, color, start_pos, end_pos, width=2, dash_length=5):
    """ Draws a dashed line in simulation """
    x1, y1 = start_pos
    x2, y2 = end_pos
    dl = dash_length

    a = abs(x2 - x1)
    b = abs(y2 - y1)
    c = round(math.sqrt(a ** 2 + b ** 2))
    dx = dl * a / c
    dy = dl * b / c

    xcoords = [x for x in np.arange(x1, x2, dx if x1 < x2 else -dx)]
    ycoords = [y for y in np.arange(y1, y2, dy if y1 < y2 else -dy)]

    next_coords = list(zip(xcoords[1::2], ycoords[1::2]))
    last_coords = list(zip(xcoords[0::2], ycoords[0::2]))
    for (x1, y1), (x2, y2) in zip(next_coords, last_coords):
        start = (round(x1), round(y1))
        end = (round(x2), round(y2))
        pygame.draw.line(surf, color, start, end, width)