import math
from shapely.geometry import LineString
from shapely.geometry import Point


class Robot:

    def __init__(self, x, y, radius, num_sensors, max_sensor_reach):
        self.x = x
        self.y = y
        self.radius = radius
        self.orientation = (x + radius, y)

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

        return False



