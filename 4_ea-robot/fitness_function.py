
import shapely.geometry

## Fitness function - evaluate how much dust was cleaned

## Goal
# - collision-free navigation <-> come close to walls
# - clean maximum amount of dust in each time step

## Approach
# - one evaluation function for each sensor e_s()
#   . gives higher score the closer sensor is to the wall
#   . constant score for values >= sensor reach
#   . low score for collision

# - one evaluation function for dust cleaned e_d()
#   . calculate area of dust cleaned by movement
#   . calculate total area

# - combine functions: score = max_i (e_s_i()) * e_d()
# prefer distances close to walls?

def robot_fitness(robot, wall_length):
    total_area = shapely.geometry.box(0, 0, wall_length, wall_length).area
    cleaned_area = robot.score
    sensor_ev = robot.sensor_score

    # if sensor_ev == 0:
    #     return 0
    #
    # else:
    #     return cleaned_area * 1 / sensor_ev #/ total_area

    # evtl.: multiply with constants
    return  cleaned_area / total_area
