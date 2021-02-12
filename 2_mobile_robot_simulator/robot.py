
class Robot:

    def __init__(self, x, y, radius, max_sensor_reach):
        self.x = x
        self.y = y
        self.radius = radius
        self.orientation = (x + radius, y)

        self.v_wheel_l = 0
        self.v_wheel_r = 0

        self.max_sensor_reach = max_sensor_reach
        self.sensor_list = []

