
# configuration of autonomous robot simulator

class Configuration:

    def __init__(self):

        # environment
        self.env_width = 750
        self.env_height = self.env_width

        # walls
        self.wall_length = 600
        self.wall_thickness = 6
        self.wall_color = (204, 0, 102)
        self.room_shape = 'square'  # square, rectangle, rectangle_double, trapezoid, trapezoid_double

        # robot
        self.x = self.env_width / 2
        self.y = self.env_height / 2
        #self.v = 0.5
        self.v_max = 15
        self.radius = self.env_width / 20
        self.num_sensors = 12
        self.max_sensor_reach = 4 * self.radius
        self.robot_color = (153, 204, 255)
        self.delta_t = 0.1
        self.path_steps = 10  #20
        self.path_color = (204, 229, 255)
        self.position_initialization = "center"  # center, corner

        # evolutionary algorithm
        self.n_individuals = 30
        self.max_n_generations = 100
        self.n_best_percentage = 0.8
        self.crossover_percentage = 0.2
        self.mutation_percentage = 0.05
        self.termination_threshold = 5

        # ANN
        self.hidden_dim = 4

        # DON'T CHANGE ANYTHING BELOW
        # position initialization for different rooms
        if self.room_shape == "square":
            if self.position_initialization == "center":
                # square - center
                self.x = self.env_width / 2
                self.y = self.env_height / 2
            elif self.position_initialization == "corner":
                # square - corner
                self.x = self.env_width - self.radius
                self.y = self.env_height - self.radius

        if self.room_shape == "rectangle":
            if self.position_initialization == "center":
                self.x = self.env_width / 2
                self.y = self.env_height / 2
            elif self.position_initialization == "corner":
                self.x = self.env_width * 0.8 - (self.env_width - self.wall_length)/2 - self.radius
                self.y = (self.env_height - self.wall_length)/2 + self.radius

        if self.room_shape == "rectangle_double":
            if self.position_initialization == "center":
                self.x = self.env_width / 2 - 150 - self.radius
                self.y = self.env_height / 2
            elif self.position_initialization == "corner":
                self.x = self.env_width * 0.8 - (self.env_width - self.wall_length)/2 - self.radius
                self.y = (self.env_height - self.wall_length)/2 + self.radius

        if self.room_shape == "trapezoid":
            if self.position_initialization == "center":
                self.x = self.env_width / 2
                self.y = self.env_height / 2
            elif self.position_initialization == "corner":
                self.x = self.env_width - (self.env_width - self.wall_length)/2 - self.radius
                self.y = self.env_height * 0.8 - (self.env_height - self.wall_length)/2

        if self.room_shape == "trapezoid_double":
            if self.position_initialization == "center":
                self.x = self.env_width / 2 - 150 - self.radius
                self.y = self.env_height / 2
            elif self.position_initialization == "corner":
                self.x = self.env_width - (self.env_width - self.wall_length) / 2 - self.radius
                self.y = self.env_height * 0.8 - (self.env_height - self.wall_length) / 2
