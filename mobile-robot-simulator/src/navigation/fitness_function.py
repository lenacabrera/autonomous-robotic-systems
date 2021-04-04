import shapely.geometry


def get_fitness(conf, robot):
    if conf.fitness_strategy == 0:
        return evaluate_covered_area(robot, conf.wall_length)
    if conf.fitness_strategy == 1:
        return evaluate_covered_area_and_avoid_collisions(robot, conf.wall_length)


def evaluate_covered_area(robot, wall_length):
    """ Evaluate area covered / visited by the robot """

    covered_area = robot.fitness_score
    total_area = shapely.geometry.box(0, 0, wall_length, wall_length).area

    return covered_area / total_area


def evaluate_covered_area_and_avoid_collisions(robot, wall_length):
    """ Evaluate area covered / visited by the robot and reward collision-free navigation """

    covered_area = robot.fitness_score
    total_area = shapely.geometry.box(0, 0, wall_length, wall_length).area

    return (covered_area / total_area) * robot.collision_score
