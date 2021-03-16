import pygame
import sys
import math
import numpy as np
from shapely.geometry import Point
from robot import Robot


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


def draw_landmarks(screen, landmarks, visible_landmarks, robot):
    for landmark in landmarks:
        pygame.draw.circle(surface=screen, color=(0, 0, 0), center=(landmark.x, landmark.y), radius=5)

    for visible_landmark in visible_landmarks:
        # solid line
        # pygame.draw.line(surface=screen, color=(0, 158, 0), width=2,
        #                  start_pos=(robot.x, robot.y), end_pos=(visible_landmark.x, visible_landmark.y))

        # dashed line
        draw_dashed_line(screen, color=(0, 158, 0),
                         start_pos=(robot.x, robot.y), end_pos=(visible_landmark.x, visible_landmark.y))


def draw_dashed_line(surf, color, start_pos, end_pos, width=2, dash_length=5):
    x1, y1 = start_pos
    x2, y2 = end_pos
    dl = dash_length

    # if start_pos[0] == end_pos[0]:
    #     ycoords = [y for y in range(y1, y2, dl if y1 < y2 else -dl)]
    #     xcoords = [x1] * len(ycoords)
    # elif start_pos[1] == end_pos[1]:
    #     xcoords = [x for x in range(x1, x2, dl if x1 < x2 else -dl)]
    #     ycoords = [y1] * len(xcoords)
    # else:
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


def draw_robot(screen, robot, robot_color, distance_values, draw_sensors=False):
    # body of robot
    pygame.draw.circle(surface=screen, color=robot_color, center=(robot.x, robot.y), radius=robot.radius)
    # orientation of robot
    pygame.draw.line(surface=screen, color=(0, 0, 0), width=2,
                     start_pos=(robot.x, robot.y), end_pos=robot.orientation)
    # sensors
    if draw_sensors:
        for i_sensor, (x_sensor, y_sensor) in enumerate(robot.sensor_list):
            angle = (i_sensor + 1) * 360 / robot.num_sensors
            sensor_start_x = robot.x + math.cos(angle * math.pi / 180) * robot.radius
            sensor_start_y = robot.y + math.sin(angle * math.pi / 180) * robot.radius

            sensor_length = robot.radius + distance_values[i_sensor]

            sensor_end_x = robot.x + math.cos(angle * math.pi / 180) * sensor_length
            sensor_end_y = robot.y + math.sin(angle * math.pi / 180) * sensor_length

            pygame.draw.line(surface=screen, color=(255, 0, 0), width=1,
                             start_pos=(sensor_start_x, sensor_start_y), end_pos=(sensor_end_x, sensor_end_y))

            # display sensor values
            text = str(int(distance_values[i_sensor]))
            textsurface = myfont.render(text, False, (0, 0, 0))
            screen.blit(textsurface, (sensor_end_x - 7 + math.cos(angle * math.pi / 180) * robot.radius / 4,
                                      sensor_end_y - 7 + math.sin(angle * math.pi / 180) * robot.radius / 4))

    # display motor speed values
    text = str(int(robot.v_wheel_r))
    textsurface = myfont.render(text, False, (0, 0, 0))
    screen.blit(textsurface, (robot.x + np.cos(robot.line_angle + math.pi / 2) * robot.radius / 2,
                              robot.y + np.sin(robot.line_angle + math.pi / 2) * robot.radius / 2))

    text = str(int(robot.v_wheel_l))
    textsurface = myfont.render(text, False, (0, 0, 0))
    screen.blit(textsurface, (robot.x + np.cos(robot.line_angle + 3 * math.pi / 2) * robot.radius / 2,
                              robot.y + np.sin(robot.line_angle + 3 * math.pi / 2) * robot.radius / 2))


def draw_robot_way(robot):
    robot_positions = robot.positions

    for start in range(len(robot_positions) - 1):
        pygame.draw.line(surface=screen, color=(0, 0, 0), width=2,
                         start_pos=(robot_positions[start][0], robot_positions[start][1]),
                         end_pos=(robot_positions[start + 1][0], robot_positions[start + 1][1]))


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


def init_landmarks(env_width, env_height, wall_length):
    dist_l_r = (env_width - wall_length) / 2  # distance to frame, left and right
    dist_t_b = (env_height - wall_length) / 2  # distance to frame, top and bottom

    room_l = dist_l_r
    room_r = env_width - dist_l_r
    room_t = dist_t_b
    room_b = env_height - dist_t_b

    f = 0.17 * wall_length
    positions = [
        (room_l + 1.2 * f, room_t + 1.5 * f),
        (room_l + 2.5 * f, room_t + 3.4 * f),
        (room_l + 4.3 * f, room_t + 2.9 * f),
        (room_l + 5.3 * f, room_t + 3.9 * f),
        (room_l + 4.8 * f, room_t + 5.0 * f),
        (room_l + 3.9 * f, room_t + 3.7 * f),

    ]

    landmarks = []
    for position in positions:
        landmarks.append(Point(position))

    return landmarks


if __name__ == '__main__':
    # environment
    env_width = 750
    env_height = env_width
    wall_length = 600
    wall_thickness = 6
    wall_color = (204, 0, 102)

    # robot
    x = env_width / 2
    y = env_height / 2
    o = 0.1  # TODO
    v = 0.5
    v_max = 15
    radius = env_width / 20
    num_sensors = 12
    max_sensor_reach = 2 * radius
    robot_color = (153, 204, 255)
    robot = Robot(x, y, radius, num_sensors, max_sensor_reach)

    # environment
    walls = init_walls_coordinates(env_width, env_height, wall_length)
    landmarks = init_landmarks(env_width, env_height, wall_length)

    # initialize pygame
    pygame.init()
    screen = pygame.display.set_mode([env_width, env_height])
    clock = pygame.time.Clock()

    # initialize display numbers
    myfont = pygame.font.SysFont('Arial', 14)
    screen.fill((255, 255, 255))

    # initialize timer in order to provide constant timer-events
    timer_event = pygame.USEREVENT + 1
    time = 250  # TODO: fine-tuning
    pygame.time.set_timer(timer_event, time)

    # run animation
    go = True
    while go:
        for event in pygame.event.get():
            # for key / action
            if event.type == pygame.QUIT:
                # termination
                sys.exit()

            pressed_keys = pygame.key.get_pressed()

            # TODO control robot with the following keys
            """
            W -> increment v
            S -> decrement v 
            D -> increment omega
            A -> decrement omega
            X -> stop (stays the same)
            """

            # if (robot.v_wheel_l + robot.v_wheel_r) / 2 < v_max:
            if pressed_keys[pygame.K_w]:
                if (robot.v_wheel_l + robot.v_wheel_r) / 2 < v_max:
                    # increase velocity
                    robot.v_wheel_l += v/2
                    robot.v_wheel_r += v/2

            if pressed_keys[pygame.K_s]:
                if (robot.v_wheel_l + robot.v_wheel_r) / 2 > -v_max:
                    # decrease velocity
                    robot.v_wheel_l -= v/2
                    robot.v_wheel_r -= v/2

            if pressed_keys[pygame.K_d]:
                # increase omega (rotation)
                robot.omega += o
            if pressed_keys[pygame.K_a]:
                # decrease omega (rotation)
                robot.omega -= o

            if pressed_keys[pygame.K_x]:
                # stop
                robot.v = 0

            if (robot.v_wheel_l + robot.v_wheel_r) / 2 < v_max:

                if pressed_keys[pygame.K_w]:
                    robot.v_wheel_l += v
                if pressed_keys[pygame.K_o]:
                    robot.v_wheel_r += v
                if pressed_keys[pygame.K_t]:
                    robot.v_wheel_l += v
                    robot.v_wheel_r += v
                if pressed_keys[pygame.K_x]:
                    robot.v_wheel_l = 0
                    robot.v_wheel_r = 0

            if (robot.v_wheel_l + robot.v_wheel_r) / 2 > - v_max:
                if pressed_keys[pygame.K_s]:
                    robot.v_wheel_l -= v
                if pressed_keys[pygame.K_l]:
                    robot.v_wheel_r -= v
                if pressed_keys[pygame.K_g]:
                    robot.v_wheel_l -= v
                    robot.v_wheel_r -= v
                if pressed_keys[pygame.K_x]:
                    robot.v_wheel_l = 0
                    robot.v_wheel_r = 0

            # update screen by providing timer-event
            if event.type == timer_event:
                # update robot position with delta_t = 0.1
                robot.set_new_position(0.1)

                # check for collision
                robot.robot_is_crossing_wall(walls)

                # sensor value update
                robot.update_sensors()
                sensor_d = robot.get_sensor_distance_values(walls)

                visible_landmarks = robot.check_landmarks_in_sight(landmarks)

                # clear screen
                screen.fill((255, 255, 255))

                # draw scene
                draw_walls(screen, walls, wall_thickness, wall_color)
                # for testing of sensor circle and visibility of landmarks
                # pygame.draw.circle(surface=screen, color=(0, 158, 0), center=(robot.x, robot.y), radius=robot.max_sensor_reach+robot.radius, width=2)
                draw_robot_way(robot)
                draw_robot(screen, robot, robot_color, sensor_d, draw_sensors=False)
                draw_landmarks(screen, landmarks, visible_landmarks, robot)

                # update
                pygame.display.update()
