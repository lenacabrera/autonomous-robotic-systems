import numpy as np
import math
from shapely.geometry import LineString
from shapely.geometry import Point
from shapely.ops import cascaded_union


class Robot:

    def __init__(self, x, y, radius, num_sensors, max_sensor_reach):
        self.x = x
        self.y = y
        self.radius = radius
        self.orientation = [x + radius, y]  # initialization: line to the right
        self.line_angle = 0

        self.x_prev = x
        self.y_prev = y

        self.v_wheel_l = 0
        self.v_wheel_r = 0

        self.num_sensors = num_sensors
        self.max_sensor_reach = max_sensor_reach
        self.sensor_list = []
        self.init_sensors()

        self.positions = [(self.x, self.y)]
        self.circles = [Point(self.x, self.y).buffer(self.radius), Point(self.x, self.y).buffer(self.radius)]

        self.sensor_score = 10
        self.score = 0

    def init_sensors(self):
        self.update_sensors()

    def update_score(self):
        # for i in range(len(self.circles) - 2):
        polygon = cascaded_union(self.circles[:-2])
        intersection = polygon.intersection(self.circles[-1]).area
        self.score += (self.circles[-1].area - intersection)

    def update_sensors(self):
        # update coordinates of sensors after robot has moved
        sensors = []
        for sensor_id in range(self.num_sensors):
            angle = (sensor_id + 1) * 360 / self.num_sensors
            length_sensor_line = self.radius + self.max_sensor_reach
            radians = math.atan2((self.orientation[1] - self.y), (self.orientation[0] - self.x))
            x_sensor = self.x + length_sensor_line * math.cos(angle * math.pi / 180 + radians)
            y_sensor = self.y + length_sensor_line * math.sin(angle * math.pi / 180 + radians)
            sensors.append((x_sensor, y_sensor))

        self.sensor_list = sensors

    def robot_is_crossing_wall(self, walls):
        # check if robot crosses the wall and if so update position
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

        self.positions.append((self.x, self.y))
        self.circles.append(Point(self.x, self.y).buffer(self.radius))
        self.update_score()

        if intersection_left.is_empty and intersection_right.is_empty and intersection_top.is_empty and intersection_bottom.is_empty:
            return
        else:
            # update orientation if robot bumped into wall
            radians = math.atan2(self.orientation[1] - self.y_prev, self.orientation[0] - self.x_prev)
            self.orientation = (self.x + (self.radius * math.cos(radians)), self.y + (self.radius * math.sin(radians)))

    def get_sensor_distance_values(self, walls):
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
                            # if robot hit the wall, make score zero
                            self.sensor_score = 0

            distance_values.append(min(sensor_distances))

        return distance_values

    def set_new_position(self, delta_t, v_left, v_right):
        self.v_wheel_l = v_left
        self.v_wheel_r = v_right

        # for collision detection: save previous position
        self.x_prev = self.x
        self.y_prev = self.y

        # # calculate angular velocity: omega
        omega = (self.v_wheel_l - self.v_wheel_r) / 2

        # # calculate distance to ICC: R
        if self.v_wheel_l != self.v_wheel_r:
            R = self.radius * (self.v_wheel_l + self.v_wheel_r) / (self.v_wheel_l - self.v_wheel_r)
        else:
            R = 100

        # # calculate angle of robot relative to x-axis (horizontal axis): Theta
        # vector_1 pointing in the direction of the x axis, vector_2 in the direction of the robot
        vector_1 = [1, 0]
        vector_2 = [self.orientation[0] - self.x, self.orientation[1] - self.y]
        # normalization of the vectors
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        # dot product and determinant
        dot = unit_vector_1[0] * unit_vector_2[0] + unit_vector_1[1] * unit_vector_2[1]
        det = unit_vector_1[0] * unit_vector_2[1] - unit_vector_2[0] * unit_vector_1[1]
        # the angle Theta is calculated with arctan2 as angles between 0 and 360 degrees are needed
        Theta = np.arctan2(det, dot)

        # # calculate ICC
        ICCx = self.x - R * np.sin(Theta)
        ICCy = self.y + R * np.cos(Theta)

        # # equation (5) slide 19
        # matrix 1
        m1 = np.array([[np.cos(omega * delta_t), - np.sin(omega * delta_t), 0],
                       [np.sin(omega * delta_t), np.cos(omega * delta_t), 0],
                       [0, 0, 1]
                       ])
        # matrix 2
        m2 = np.array([[self.x - ICCx],
                       [self.y - ICCy],
                       [Theta]
                       ])
        # matrix 3
        m3 = np.array([[ICCx],
                       [ICCy],
                       [omega * delta_t]
                       ])
        # the multiplication and addition: The output is [x' y' Theta']
        if (self.v_wheel_l != self.v_wheel_r):
            new = np.matmul(m1, m2) + m3

        else:
            new = np.array([[self.x + self.v_wheel_l * np.cos(Theta)],
                            [self.y + self.v_wheel_l * np.sin(Theta)],
                            [Theta]
                            ])

        # # updating the x and the y coordinate of the robot
        self.x = new[0][0]
        self.y = new[1][0]

        # # updating the orientation: the point on the border of the circle
        self.orientation = (self.x + np.cos(new[2][0]) * self.radius,
                            self.y + np.sin(new[2][0]) * self.radius)

        self.line_angle = new[2][0]