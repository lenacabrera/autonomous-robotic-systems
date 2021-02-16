import numpy as np
import math
from shapely.geometry import LineString
from shapely.geometry import Point


class Robot:

    def __init__(self, x, y, radius, num_sensors, max_sensor_reach):
        self.x = x
        self.y = y
        self.radius = radius
        self.orientation = (x + radius, y)  # initialization: line to the right

        self.v_wheel_l = 0
        self.v_wheel_r = 0

        self.num_sensors = num_sensors
        self.max_sensor_reach = max_sensor_reach
        self.sensor_list = []
        self.init_sensors()


    def init_sensors(self):
        self.update_sensors()

    def update_sensors(self):
        # update coordinates of sensors after robot has moved
        sensors = []
        for sensor_id in range(self.num_sensors):
            angle = (sensor_id + 1) * 360 / self.num_sensors
            length_sensor_line = self.radius + self.max_sensor_reach
            x_sensor = self.x + length_sensor_line * math.cos(angle * math.pi / 180)
            y_sensor = self.y + length_sensor_line * math.sin(angle * math.pi / 180)
            sensors.append((x_sensor, y_sensor))

        self.sensor_list = sensors


    def robot_is_crossing_wall(self, walls):

        for wall_name, wall_coord in walls.items():
            position = Point(self.x, self.y)
            circle = position.buffer(self.radius).boundary
            line = LineString([wall_coord[0], wall_coord[1]])
            intersection = circle.intersection(line)
            # print(intersection)

            if not intersection.is_empty:
                return True

        # TODO check for all walls whether robot is outside of wall frame

        return False


    def set_new_position(self, delta_t):
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
        new = np.matmul(m1, m2) + m3

        # # updating the x and the y coordinate of the robot
        self.x = new[0][0]
        self.y = new[1][0]

        # # updating the orientation: the point on the border of the circle
        self.orientation = (self.x + np.cos(new[2][0]) * self.radius,
                            self.y + np.sin(new[2][0]) * self.radius)

