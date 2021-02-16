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
                     start_pos=(robot.x, robot.y), end_pos=robot.orientation)
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
    env_width = 500
    env_height = env_width
    wall_length = 400
    wall_thickness = 6
    wall_color = (204, 0, 102)

    # robot
    x = env_width / 2
    y = env_height / 2
    v = 0.1  #TODO: fine-tuning
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

    # initialize display velocity
    myfont = pygame.font.SysFont('Arial', 14)
    screen.fill((255, 255, 255))

    # initialize timer in order to provide constant timer-events
    timer_event = pygame.USEREVENT + 1
    time = 2500  #TODO: fine-tuning
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

            # simple robot movement
            # if pressed_keys[pygame.K_UP]:
            #     robot.y -= v
            # if pressed_keys[pygame.K_DOWN]:
            #     robot.y += v
            # if pressed_keys[pygame.K_LEFT]:
            #     robot.x -= v
            # if pressed_keys[pygame.K_RIGHT]:
            #     robot.x += v

            # change velocity one wheel
            if pressed_keys[pygame.K_w]:
                 robot.v_wheel_l +=v
            if pressed_keys[pygame.K_s]:
                robot.v_wheel_l -=v
            if pressed_keys[pygame.K_o]:
                robot.v_wheel_r +=v
            if pressed_keys[pygame.K_l]:
                robot.v_wheel_r -=v

            # change velocity both wheels
            if pressed_keys[pygame.K_x]:
                robot.v_wheel_l = 0
                robot.v_wheel_r = 0
            if pressed_keys[pygame.K_t]:
                robot.v_wheel_l += v
                robot.v_wheel_r += v
            if pressed_keys[pygame.K_g]:
                robot.v_wheel_l -= v
                robot.v_wheel_r -= v

            # update screen by providing timer-event
            elif event.type == timer_event:
                robot.set_new_position(1)
                robot.update_sensors()
                if robot.robot_is_crossing_wall(walls):
                    print("Robot bumped into wall")

                # clear screen
                screen.fill((255, 255, 255))

                # print velocities
                text = 'v_left: ' + str('%.3f'%(robot.v_wheel_l)) + '   v_right: ' + str('%.3f'%(robot.v_wheel_r))
                textsurface = myfont.render(text, False, (0, 0, 0))
                screen.blit(textsurface, (env_width / 3, wall_length + (env_height - wall_length)/1.5))

                # draw scene
                draw_walls(screen, walls, wall_thickness, wall_color)
                draw_robot(screen, robot, robot_color, draw_sensors=True) # gestrichelt? TODO

                pygame.display.update()
                # clock.tick(120)
