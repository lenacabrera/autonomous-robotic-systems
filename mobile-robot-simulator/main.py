import pygame
import sys
import robot
import environment
import localization
import animation


class Configuration:

    def __init__(self):
        # environment
        self.env_width = 750
        self.env_height = self.env_width
        self.room_shape = 'square'  # square, rectangle, rectangle_double, trapezoid, trapezoid_double
        self.wall_length = 600
        self.wall_thickness = 6
        self.wall_color = (196, 196, 196)  # (204, 0, 102)

        # robot
        self.robot_type = "v"  # d -> differential, v - > velocity
        self.x = self.env_width / 2
        self.y = self.env_height / 2
        self.position_initialization = "center"  # robot's starting position: center, corner
        self.o = 0.05
        self.v = 0.5
        self.v_max = 15
        self.radius = self.env_width / 20
        self.robot_color = (0, 153, 255)  # (153, 204, 255)
        self.num_sensors = 12
        self.max_sensor_reach = 2 * self.radius
        self.visible_sensor_beams = True

        # animation
        self.delta_t = 1  # time step
        self.uncertainty_update = 20

        # task
        self.task = "l"  # l -> localization, e -> evolution

        # localization with kalman filter, randomly drawing from normal distribution (Gaussian) with
        self.kf_mean = 1
        self.kf_std_Sigma = 0.01
        self.kf_std_R = 0.7
        self.kf_std_Q = 0.3
        self.kf_std_z = 0.2
        self.kf_uncertainty_growth = 0.1



if __name__ == '__main__':

    # load configuration
    conf = Configuration()

    # setup environment
    walls = environment.get_walls(conf)
    landmarks = environment.get_landmarks(conf)

    # setup robot
    if conf.robot_type == "d":
        conf.delta_t *= 0.1
        robot = robot.Differential_Drive_Robot(conf)
    elif conf.robot_type == "v":
        robot = robot.Velocity_Drive_Robot(conf)

    if conf.task == "l":
        # localization
        kalman_filter = localization.KalmanFilter(conf=conf, x=robot.x, y=robot.y, theta=0)
        uncertainty_history = []
        uncertainty_increase = 0

    # initialize pygame
    pygame.init()
    clock = pygame.time.Clock()
    timer_event = pygame.USEREVENT + 1  # initialize timer in order to provide constant timer-events
    pygame.time.set_timer(timer_event, 250)
    screen = pygame.display.set_mode([conf.env_width, conf.env_height])
    screen.fill((255, 255, 255))
    font = pygame.font.SysFont('Arial', 14)

    # run simulation
    go = True
    while go:
        for event in pygame.event.get():
            # for key / action
            if event.type == pygame.QUIT:
                # termination
                sys.exit()

            robot.perform_motion(conf, pygame, robot)

            # update simulation frame
            if event.type == timer_event:
                robot.update_position(conf.delta_t)

                # check for collision
                robot.robot_is_crossing_wall(walls)
                
                
                if conf.task == "l":  # localization
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
                animation.draw_walls(pygame, screen, walls, conf.wall_thickness, conf.wall_color)
                animation.draw_landmarks(pygame, screen, landmarks)
                animation.draw_robot(pygame, screen, font, robot, walls, visible_landmarks,
                                     visible_sensor_beams=conf.visible_sensor_beams)
                animation.draw_true_robot_trajectory(pygame, screen, robot)
                animation.draw_believed_robot_trajectory(pygame, screen, kalman_filter)
                animation.draw_uncertainty_ellipses(pygame, screen, kalman_filter, conf.uncertainty_update)

                # update screen
                pygame.display.update()
