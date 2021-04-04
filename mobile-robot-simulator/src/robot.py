import numpy as np
import math
from shapely.geometry import LineString, Point
from shapely.ops import cascaded_union
from abc import ABC, abstractmethod


class Robot(ABC):

    def __init__(self, conf):
        self.radius = conf.radius

        # position
        self.x = conf.x
        self.y = conf.y
        self.x_prev = conf.x
        self.y_prev = conf.y

        # velocity
        self.v_wheel_l = 0
        self.v_wheel_r = 0
        self.v_max = conf.v_max

        # orientation
        self.orientation = [conf.x + conf.radius, conf.y]  # initialization: line to the right
        self.omega = 0
        self.theta = 0

        # sensors
        self.n_sensors = conf.n_sensors
        self.max_sensor_reach = conf.max_sensor_reach
        self.sensor_list = []
        self.init_sensors()
        # omni-directional sensor
        self.sensor_circle = Point(self.x, self.y).buffer(self.max_sensor_reach + self.radius)

        # trajectory
        self.trajectory_positions = [(self.x, self.y)]
        self.trajectory_circles = [Point(self.x, self.y).buffer(self.radius), Point(self.x, self.y).buffer(self.radius)]

        self.fitness_score = 0
        self.sensor_score = 10


        self.color = conf.robot_color

    def init_sensors(self):
        self.update_sensors()

    @abstractmethod
    def perform_motion(self, conf, pygame, use_keyboard=True):
        pass

    @abstractmethod
    def update_position(self, delta_t):
        pass

    def update_sensors(self):
        """ Update coordinates of sensor ends after robot has moved """
        sensors = []
        for sensor_id in range(self.n_sensors):
            angle = (sensor_id + 1) * 360 / self.n_sensors
            length_sensor_line = self.radius + self.max_sensor_reach
            radians = math.atan2((self.orientation[1] - self.y), (self.orientation[0] - self.x))
            x_sensor = self.x + length_sensor_line * math.cos(angle * math.pi / 180 + radians)
            y_sensor = self.y + length_sensor_line * math.sin(angle * math.pi / 180 + radians)
            sensors.append((x_sensor, y_sensor))

        self.sensor_list = sensors

    def get_sensor_distance_values(self, walls):
        """ Retrieves distances for all sensors in which they detect an object (within sensor range)"""
        distance_values = []
        for i_sensor, (x_sensor, y_sensor) in enumerate(self.sensor_list):
            sensor_distances = []
            for wall_name, wall_tuples in walls.items():
                for wall_coord in wall_tuples:
                    line_wall = LineString([wall_coord[0], wall_coord[1]])
                    line_sensor = LineString([(self.x, self.y), (x_sensor, y_sensor)])
                    intersection = line_wall.intersection(line_sensor)
                    if intersection.is_empty:
                        # if no intersection -> maximum sensor reach
                        sensor_distances.append(self.max_sensor_reach)
                        self.sensor_score *= self.max_sensor_reach / self.max_sensor_reach
                    else:
                        # if intersection
                        a = abs(self.x - intersection.x)
                        b = abs(self.y - intersection.y)
                        c = math.sqrt(math.pow(a, 2) + math.pow(b, 2)) - self.radius
                        sensor_distances.append(c)
                        self.sensor_score *= c / self.max_sensor_reach
                        if int(c) == 0:
                            # if robot hit the wall, make fitness_score zero
                            self.sensor_score = 0

            distance_values.append(min(sensor_distances))

        return distance_values

    def collision_detection(self, walls):
        # Checks if robot crosses a wall and if so update position such that robot slides along the wall
        x = self.x
        y = self.y

        # walls right of robot
        for right_wall in walls["right"]:
            line_right = LineString([(self.x_prev, self.y_prev), (self.x + self.radius, self.y)])
            wall_right = LineString([right_wall[0], right_wall[1]])
            intersection_right = wall_right.intersection(line_right)
            if not intersection_right.is_empty:
                x = intersection_right.x - self.radius

        # walls left of robot
        for left_wall in walls["left"]:
            line_left = LineString([(self.x_prev, self.y_prev), (self.x - self.radius, self.y)])
            wall_left = LineString([left_wall[0], left_wall[1]])
            intersection_left = wall_left.intersection(line_left)
            if not intersection_left.is_empty:
                x = intersection_left.x + self.radius

        # walls above robot
        for top_wall in walls["top"]:
            line_top = LineString([(self.x_prev, self.y_prev), (self.x, self.y - self.radius)])
            wall_top = LineString([top_wall[0], top_wall[1]])
            intersection_top = wall_top.intersection(line_top)
            if not intersection_top.is_empty:
                y = intersection_top.y + self.radius

        # walls below robot
        for bottom_wall in walls["bottom"]:
            line_bottom = LineString([(self.x_prev, self.y_prev), (self.x, self.y + self.radius)])
            wall_bottom = LineString([bottom_wall[0], bottom_wall[1]])
            intersection_bottom = wall_bottom.intersection(line_bottom)
            if not intersection_bottom.is_empty:
                y = intersection_bottom.y - self.radius

        self.x = x
        self.y = y

        self.trajectory_positions.append((self.x, self.y))
        self.trajectory_circles.append(Point(self.x, self.y).buffer(self.radius))
        self.update_score()

        if intersection_left.is_empty and intersection_right.is_empty and intersection_top.is_empty and intersection_bottom.is_empty:
            return
        else:
            # update orientation if robot bumped into wall
            radians = math.atan2(self.orientation[1] - self.y_prev, self.orientation[0] - self.x_prev)
            self.orientation = (self.x + (self.radius * math.cos(radians)), self.y + (self.radius * math.sin(radians)))


    def check_landmarks_in_sight(self, landmarks):
        # Checks which landmarks are in sight of the robot
        visible_landmarks = []
        distances = []
        bearings = []
        for landmark in landmarks:
            if not self.sensor_circle.intersection(landmark).is_empty:
                visible_landmarks.append(landmark)
                distances.append(LineString([(self.x, self.y), (landmark.x, landmark.y)]).length)
                radian_r_l = math.atan2((landmark.y - self.orientation[1]), (landmark.x - self.orientation[0]))
                radian_l_env = math.atan2((landmark.y - 0), (landmark.x - 1))
                bearing = radian_l_env - radian_r_l
                bearings.append(bearing)

        return visible_landmarks, distances, bearings


    def update_score(self):
        """ Updates fitness score of robot """
        polygon = cascaded_union(self.trajectory_circles[:-2])
        intersection = polygon.intersection(self.trajectory_circles[-1]).area
        self.fitness_score += (self.trajectory_circles[-1].area - intersection)


class Differential_Drive_Robot(Robot):

    def perform_motion(self, conf, pygame, use_keyboard=True):
        """ Performs motion of robot.
            Control robot with the following keys:
            W -> increment left wheel motor speed
            S -> decrement left wheel motor speed
            O -> increment right wheel motor speed
            L -> decrement right wheel motor speed
            T -> increment both wheels' motor speed
            G -> decrement both wheels' motor speed
            X -> set both wheels' motor speed to zero
        """

        if use_keyboard:
            pressed_keys = pygame.key.get_pressed()

            if (self.v_wheel_l + self.v_wheel_r) / 2 < self.v_max:

                if pressed_keys[pygame.K_w]:
                    self.v_wheel_l += conf.v
                if pressed_keys[pygame.K_o]:
                    self.v_wheel_r += conf.v
                if pressed_keys[pygame.K_t]:
                    self.v_wheel_l += conf.v
                    self.v_wheel_r += conf.v
                if pressed_keys[pygame.K_x]:
                    self.v_wheel_l = 0
                    self.v_wheel_r = 0

            if (self.v_wheel_l + self.v_wheel_r) / 2 > - self.v_max:
                if pressed_keys[pygame.K_s]:
                    self.v_wheel_l -= conf.v
                if pressed_keys[pygame.K_l]:
                    self.v_wheel_r -= conf.v
                if pressed_keys[pygame.K_g]:
                    self.v_wheel_l -= conf.v
                    self.v_wheel_r -= conf.v
                if pressed_keys[pygame.K_x]:
                    self.v_wheel_l = 0
                    self.v_wheel_r = 0

    def update_position(self, delta_t, v_left=None, v_right=None):
        """ Updates the position of the robot based on the current velocities of the two wheels """

        if v_left is not None:
            self.v_wheel_l = v_left
        if v_right is not None:
            self.v_wheel_r = v_right

        # save previous position for collision detection
        self.x_prev = self.x
        self.y_prev = self.y

        # angular velocity -> omega
        omega = (self.v_wheel_l - self.v_wheel_r) / 2

        # distance to ICC (Instantaneous Center of Curvature) -> R
        if self.v_wheel_l != self.v_wheel_r:
            R = self.radius * (self.v_wheel_l + self.v_wheel_r) / (self.v_wheel_l - self.v_wheel_r)
        else:
            R = 100

        # orientation (angle) of robot relative to x-axis (horizontal axis) -> theta
        vector_1 = [1, 0]  # points in direction of x axis
        vector_2 = [self.orientation[0] - self.x, self.orientation[1] - self.y]  # points in direction of robot
        # normalize vectors
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        # dot product and determinant
        dot = unit_vector_1[0] * unit_vector_2[0] + unit_vector_1[1] * unit_vector_2[1]
        det = unit_vector_1[0] * unit_vector_2[1] - unit_vector_2[0] * unit_vector_1[1]
        # use arctan2 to calculate angle between 0 and 360 degrees are
        theta = np.arctan2(det, dot)

        # ICC (Instantaneous Center of Curvature)
        ICC_x = self.x - R * np.sin(theta)
        ICC_y = self.y + R * np.cos(theta)

        rotation = np.array([[np.cos(omega * delta_t), - np.sin(omega * delta_t), 0],
                             [np.sin(omega * delta_t), np.cos(omega * delta_t), 0],
                             [0, 0, 1]])

        velocity = np.array([[self.x - ICC_x],
                             [self.y - ICC_y],
                             [theta]])

        offset = np.array([[ICC_x],
                           [ICC_y],
                           [omega * delta_t]])

        # new robot state (position: x and y, orientation: theta])
        if (self.v_wheel_l != self.v_wheel_r):
            new_robot_state = np.matmul(rotation, velocity) + offset
        else:
            new_robot_state = np.array([[self.x + self.v_wheel_l * np.cos(theta)],
                                        [self.y + self.v_wheel_l * np.sin(theta)],
                                        [theta]])

        # update state of robot
        self.x = new_robot_state[0][0]
        self.y = new_robot_state[1][0]
        self.theta = new_robot_state[2][0]

        # update orientation (point) of robot (point on the border of the circle)
        self.orientation = (self.x + np.cos(new_robot_state[2][0]) * self.radius,
                            self.y + np.sin(new_robot_state[2][0]) * self.radius)

        self.update_sensors()
        self.sensor_circle = Point(self.x, self.y).buffer(self.radius + self.max_sensor_reach)


class Velocity_Drive_Robot(Robot):

    def perform_motion(self, conf, pygame, use_keyboard=True):
        """ Performs motion of robot.
            Control robot with the following keys:
            W -> increment velocity
            S -> decrement velocity
            D -> increment rotation (turn right)
            A -> decrement rotation (turn left)
            X -> set velocity to zero (stop)
        """

        if use_keyboard:
            pressed_keys = pygame.key.get_pressed()

            if pressed_keys[pygame.K_w]:
                if (self.v_wheel_l + self.v_wheel_r) / 2 < self.v_max:
                    # increase velocity
                    self.v_wheel_l += conf.v / 2
                    self.v_wheel_r += conf.v / 2

            if pressed_keys[pygame.K_s]:
                if (self.v_wheel_l + self.v_wheel_r) / 2 > -self.v_max:
                    # decrease velocity
                    self.v_wheel_l -= conf.v / 2
                    self.v_wheel_r -= conf.v / 2

            if pressed_keys[pygame.K_d]:
                # increase omega (rotation to the right)
                self.omega += conf.o
            if pressed_keys[pygame.K_a]:
                # decrease omega (rotation to the left)
                self.omega -= conf.o

            if pressed_keys[pygame.K_x]:
                # stop
                self.v_wheel_l = 0
                self.v_wheel_r = 0
                self.omega = 0

    def update_position(self, delta_t):
        """ Updates the position of the robot based on the current velocity """

        # save previous position for collision detection
        self.x_prev = self.x
        self.y_prev = self.y

        # orientation (angle) of robot relative to x-axis (horizontal axis) -> theta
        vector_1 = [1, 0]  # points in direction of x axis
        vector_2 = [self.orientation[0] - self.x, self.orientation[1] - self.y]  # points in direction of robot
        # normalize vectors
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        # dot product and determinant
        dot = unit_vector_1[0] * unit_vector_2[0] + unit_vector_1[1] * unit_vector_2[1]
        det = unit_vector_1[0] * unit_vector_2[1] - unit_vector_2[0] * unit_vector_1[1]
        # use arctan2 to calculate angle between 0 and 360 degrees are
        theta = np.arctan2(det, dot)

        current_state = np.array([[self.x],
                                  [self.y],
                                  [theta]
                                  ])

        rotation = np.array([[delta_t * np.cos(theta), 0],
                             [delta_t * np.sin(theta), 0],
                             [0, delta_t]
                             ])

        velocity = np.array([[(self.v_wheel_l + self.v_wheel_r) / 2],
                             [self.omega],
                             ])

        new_robot_state = current_state + np.matmul(rotation, velocity)

        # update state of robot
        self.x = new_robot_state[0][0]
        self.y = new_robot_state[1][0]
        self.theta = new_robot_state[2][0]

        # update orientation (point) of robot (point on the border of the circle)
        self.orientation = (self.x + np.cos(new_robot_state[2][0]) * self.radius,
                            self.y + np.sin(new_robot_state[2][0]) * self.radius)

        self.update_sensors()
        self.sensor_circle = Point(self.x, self.y).buffer(self.radius + self.max_sensor_reach)
