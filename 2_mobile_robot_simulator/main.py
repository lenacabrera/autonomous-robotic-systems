import pygame
import sys


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
    pygame.draw.line(surface=screen, color=(0, 0, 0), start_pos=(x, y), end_pos=(x + radius, y), width=2)


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
    v = 6
    radius = env_width / 25
    robot_color = (153,204,255)

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
                y -= v
            if pressed_keys[pygame.K_DOWN]:
                y += v
            if pressed_keys[pygame.K_LEFT]:
                x -= v
            if pressed_keys[pygame.K_RIGHT]:
                x += v

            # clear screen
            screen.fill((255,255,255))

            draw_walls(screen, env_width, env_height, wall_length, wall_thickness, wall_color)
            draw_robot(screen, x, y, radius, robot_color)
            pygame.display.update()
            clock.tick(60)
