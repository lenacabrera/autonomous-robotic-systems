import pygame
import sys
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


def draw_robot(screen, robot, robot_color, draw_sensors=False):
    # body of robot
    pygame.draw.circle(surface=screen, color=robot_color, center=(robot.x, robot.y), radius=robot.radius)
    # orientation of robot
    pygame.draw.line(surface=screen, color=(0, 0, 0), width=2,
                     start_pos=(robot.x, robot.y), end_pos=(robot.x + robot.radius, robot.y))
    # sensors
    if draw_sensors:
        for (x_sensor, y_sensor) in robot.sensor_list:
            pygame.draw.line(surface=screen, color=(0, 0, 0), width=2,
                             start_pos=(robot.x, robot.y), end_pos=(x_sensor, y_sensor))


def init_walls_coordinates(env_width, env_height, wall_length):

    # wall frame distance
    dist_l_r = (env_width - wall_length) / 2  # distance to frame, left and right
    dist_t_b = (env_height - wall_length) / 2  # distance to frame, top and bottom


    left_start = (dist_l_r, dist_t_b)
    left_end = (dist_l_r, dist_t_b + wall_length)

    top_start = (dist_l_r, dist_t_b + wall_length)
    top_end = (dist_l_r + wall_length, dist_t_b + wall_length)

    right_start = (dist_l_r + wall_length, dist_t_b + wall_length)
    right_end = (dist_l_r + wall_length, dist_t_b)

    bottom_start = (dist_l_r, dist_t_b)
    bottom_end = (dist_l_r + wall_length, dist_t_b)

    walls = {
        'left': [left_start, left_end],
        'top': [top_start, top_end],
        'right': [right_start, right_end],
        'bottom': [bottom_start, bottom_end],
    }

    return walls

if __name__ == '__main__':

    # environment
    env_width = 1000
    env_height = env_width
    wall_length = 800
    wall_thickness = 6
    wall_color = (204, 0, 102)

    # robot
    x = env_width / 2
    y = env_height / 2
    v = 20
    radius = env_width / 25
    num_sensors = 12
    max_sensor_reach = 2 * radius
    robot_color = (153,204,255)
    robot = Robot(x, y, radius, num_sensors, max_sensor_reach)

    walls = init_walls_coordinates(env_width, env_height, wall_length)

    # initialize pygame
    pygame.init()
    screen = pygame.display.set_mode([env_width, env_height])
    clock = pygame.time.Clock()

    # run animation
    go = True
    while go:
        for event in pygame.event.get():
            # for key / action
            if event.type == pygame.QUIT:
                # termination
                sys.exit()

            pressed_keys = pygame.key.get_pressed()
            if pressed_keys[pygame.K_UP]:
                robot.y -= v
                robot.update_sensors()
            if pressed_keys[pygame.K_DOWN]:
                robot.y += v
                robot.update_sensors()
            if pressed_keys[pygame.K_LEFT]:
                robot.x -= v
                robot.update_sensors()
            if pressed_keys[pygame.K_RIGHT]:
                robot.x += v
                robot.update_sensors()


            if robot.robot_is_crossing_wall(walls):
                print("Robot bumped into wall")

            # clear screen
            screen.fill((255,255,255))

            draw_walls(screen, walls, wall_thickness, wall_color)
            draw_robot(screen, robot, robot_color, draw_sensors=True)
            pygame.display.update()
            clock.tick(60)
