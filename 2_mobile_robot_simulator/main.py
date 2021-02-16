import pygame
import sys
from robot import Robot


def draw_walls(screen, env_width, env_height, wall_length, wall_thickness, wall_color):

    # wall frame distance
    dist_l_r = (env_width - wall_length) / 2  # distance to frame, left and right
    dist_t_b = (env_height - wall_length) / 2  # distance to frame, top and bottom

    # left
    pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                     start_pos=(dist_l_r, dist_t_b),
                     end_pos=(dist_l_r, dist_t_b + wall_length))

    # top
    pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                     start_pos=(dist_l_r, dist_t_b + wall_length),
                     end_pos=(dist_l_r + wall_length, dist_t_b + wall_length))
    # right
    pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                     start_pos=(dist_l_r + wall_length, dist_t_b + wall_length),
                     end_pos=(dist_l_r + wall_length, dist_t_b))

    # bottom
    pygame.draw.line(surface=screen, color=wall_color, width=wall_thickness,
                     start_pos=(dist_l_r, dist_t_b),
                     end_pos=(dist_l_r + wall_length, dist_t_b))


def draw_robot(screen, x, y, radius, robot_color):

    # body of robot
    pygame.draw.circle(surface=screen, color=robot_color, center=(x, y), radius=radius)
    # orientation of robot
    pygame.draw.line(surface=screen, color=(0, 0, 0), start_pos=(x, y), end_pos=robot.orientation, width=2)


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
    max_sensor_reach = 2 * radius
    robot_color = (153,204,255)
    robot = Robot(x, y, radius, max_sensor_reach)

    # initialize pygame
    pygame.init()
    screen = pygame.display.set_mode([env_width, env_height])
    clock = pygame.time.Clock()

    # initialize display velocity
    myfont = pygame.font.SysFont('Arial', 14)
    screen.fill((255, 255, 255))

    # initialize timer in order to provide constant timer-events
    timer_event = pygame.USEREVENT + 1
    time = 250  #TODO: fine-tuning
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

                # clear screen
                screen.fill((255, 255, 255))

                # print velocities
                text = 'v_left: ' + str('%.3f'%(robot.v_wheel_l)) + '   v_right: ' + str('%.3f'%(robot.v_wheel_r))
                textsurface = myfont.render(text, False, (0, 0, 0))
                screen.blit(textsurface, (env_width / 3, wall_length + (env_height - wall_length)/1.5))

                # draw scene
                draw_walls(screen, env_width, env_height, wall_length, wall_thickness, wall_color)
                draw_robot(screen, robot.x, robot.y, robot.radius, robot_color)

                pygame.display.update()
                # clock.tick(120)
