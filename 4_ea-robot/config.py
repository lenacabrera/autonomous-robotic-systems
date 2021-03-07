
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

        # robot
        self.x = self.env_width / 2
        self.y = self.env_height / 2
        self.v = 0.5
        self.v_max = 15
        self.radius = self.env_width / 20
        self.num_sensors = 12
        self.max_sensor_reach = 2 * self.radius
        self.robot_color = (153, 204, 255)
        self.delta_t = 0.1
        self.path_steps = 20 #20

        # evolutionary algorithm
        self.n_individuals = 50 #100
        self.n_iterations = 100
        self.benchmark_function = 'rastrigin'  # rastrigin, rosenbrock
        self.frame_range = [-4, 4]
        self.n_best_percentage = 0.2
        self.crossover_percentage = 0.6
        self.mutation_percentage = 0.1
        self.termination_threshold = 5

        # ANN
        self.hidden_dim = 4

        # debugging
        self.steering = "keyboard"  # keyboard, autonomous
