from shapely.geometry import Point

def get_walls(conf, test_environment=None):
    """ Retrieve coordinates of walls which are shaping the environment """

    if test_environment is not None:
        room_shape = test_environment
    else:
        room_shape = conf.room_shape

    # wall frame distance
    dist_l_r = (conf.env_width - conf.wall_length) / 2  # distance to frame, left and right
    dist_t_b = (conf.env_height - conf.wall_length) / 2  # distance to frame, top and bottom

    if room_shape == "square":

        left_start = (dist_l_r, dist_t_b)
        left_end = (dist_l_r, dist_t_b + conf.wall_length)

        top_start = (dist_l_r, dist_t_b)
        top_end = (dist_l_r + conf.wall_length, dist_t_b)

        right_start = (dist_l_r + conf.wall_length, dist_t_b)
        right_end = (dist_l_r + conf.wall_length, dist_t_b + conf.wall_length)

        bottom_start = (dist_l_r, dist_t_b + conf.wall_length)
        bottom_end = (dist_l_r + conf.wall_length, dist_t_b + conf.wall_length)

        walls = {
            'left': [(left_start, left_end)],
            'top': [(top_start, top_end)],
            'right': [(right_start, right_end)],
            'bottom': [(bottom_start, bottom_end)],
        }

        if conf.position_initialization == "center":
            # square - center
            conf.x = conf.env_width / 2
            conf.y = conf.env_height / 2
        elif conf.position_initialization == "corner":
            # square - corner
            conf.x = conf.env_width - conf.radius
            conf.y = conf.env_height - conf.radius

    elif room_shape == "rectangle":

        factor = 0.8

        left_start = (dist_l_r, dist_t_b)
        left_end = (dist_l_r, dist_t_b + conf.wall_length)

        top_start = (dist_l_r, dist_t_b)
        top_end = (dist_l_r + conf.wall_length * factor, dist_t_b)

        right_start = (dist_l_r + conf.wall_length * factor, dist_t_b)
        right_end = (dist_l_r + conf.wall_length * factor, dist_t_b + conf.wall_length)

        bottom_start = (dist_l_r, dist_t_b + conf.wall_length)
        bottom_end = (dist_l_r + conf.wall_length * factor, dist_t_b + conf.wall_length)

        walls = {
            'left': [(left_start, left_end)],
            'top': [(top_start, top_end)],
            'right': [(right_start, right_end)],
            'bottom': [(bottom_start, bottom_end)],
        }

        if conf.position_initialization == "center":
            conf.x = conf.env_width / 2
            conf.y = conf.env_height / 2
        elif conf.position_initialization == "corner":
            conf.x = conf.env_width * 0.8 - (conf.env_width - conf.wall_length) / 2 - conf.radius
            conf.y = (conf.env_height - conf.wall_length) / 2 + conf.radius

    elif room_shape == "rectangle_double":

        factor = 0.8
        difference_outer_inner = 175

        left_start = (dist_l_r, dist_t_b)
        left_end = (dist_l_r, dist_t_b + conf.wall_length)

        top_start = (dist_l_r, dist_t_b)
        top_end = (dist_l_r + conf.wall_length * factor, dist_t_b)

        right_start = (dist_l_r + conf.wall_length * factor, dist_t_b)
        right_end = (dist_l_r + conf.wall_length * factor, dist_t_b + conf.wall_length)

        bottom_start = (dist_l_r, dist_t_b + conf.wall_length)
        bottom_end = (dist_l_r + conf.wall_length * factor, dist_t_b + conf.wall_length)

        left_start_inner = (dist_l_r + difference_outer_inner, dist_t_b + difference_outer_inner)  # top
        left_end_inner = (dist_l_r + difference_outer_inner, dist_t_b + conf.wall_length - difference_outer_inner)  # bottom

        right_start_inner = (
        dist_l_r + conf.wall_length * factor - difference_outer_inner, dist_t_b + difference_outer_inner)
        right_end_inner = (
        dist_l_r + conf.wall_length * factor - difference_outer_inner, dist_t_b + conf.wall_length - difference_outer_inner)

        top_start_inner = (dist_l_r + difference_outer_inner, dist_t_b + difference_outer_inner)
        top_end_inner = (dist_l_r + conf.wall_length * factor - difference_outer_inner, dist_t_b + difference_outer_inner)

        bottom_start_inner = (dist_l_r + difference_outer_inner, dist_t_b + conf.wall_length - difference_outer_inner)
        bottom_end_inner = (
        dist_l_r + conf.wall_length * factor - difference_outer_inner, dist_t_b + conf.wall_length - difference_outer_inner)

        walls = {
            'left': [(left_start, left_end), (right_start_inner, right_end_inner)],
            'top': [(top_start, top_end), (bottom_start_inner, bottom_end_inner)],
            'right': [(right_start, right_end), (left_start_inner, left_end_inner)],
            'bottom': [(bottom_start, bottom_end), (top_start_inner, top_end_inner)],
        }

        if conf.position_initialization == "center":
            conf.x = conf.env_width / 2 - 150 - conf.radius
            conf.y = conf.env_height / 2
        elif conf.position_initialization == "corner":
            conf.x = conf.env_width * 0.8 - (conf.env_width - conf.wall_length) / 2 - conf.radius
            conf.y = (conf.env_height - conf.wall_length) / 2 + conf.radius

    elif room_shape == "trapezoid":

        left_start = (dist_l_r, dist_t_b)  # top
        left_end = (dist_l_r, dist_t_b + conf.wall_length)  # bottom

        right_start = (dist_l_r + conf.wall_length, dist_t_b + conf.wall_length * 0.2)  # top
        right_end = (dist_l_r + conf.wall_length, dist_t_b + conf.wall_length - conf.wall_length * 0.2)  # bottom

        top_start = left_start
        top_end = right_start

        bottom_start = left_end
        bottom_end = right_end

        walls = {
            'left': [(left_start, left_end)],
            'top': [(top_start, top_end)],
            'right': [(right_start, right_end)],
            'bottom': [(bottom_start, bottom_end)],
        }

        if conf.position_initialization == "center":
            conf.x = conf.env_width / 2
            conf.y = conf.env_height / 2
        elif conf.position_initialization == "corner":
            conf.x = conf.env_width - (conf.env_width - conf.wall_length) / 2 - conf.radius
            conf.y = conf.env_height * 0.8 - (conf.env_height - conf.wall_length) / 2

    if room_shape == "trapezoid_double":
        difference_outer_inner = 200

        left_start = (dist_l_r, dist_t_b)  # top
        left_end = (dist_l_r, dist_t_b + conf.wall_length)  # bottom

        left_start_inner = (dist_l_r + difference_outer_inner, dist_t_b + difference_outer_inner)  # top
        left_end_inner = (dist_l_r + difference_outer_inner, dist_t_b + conf.wall_length - difference_outer_inner)  # bottom

        right_start = (dist_l_r + conf.wall_length, dist_t_b + conf.wall_length * 0.2)  # top
        right_end = (dist_l_r + conf.wall_length, dist_t_b + conf.wall_length - conf.wall_length * 0.2)  # bottom

        right_start_inner = (dist_l_r + conf.wall_length - difference_outer_inner,
                             dist_t_b + conf.wall_length * 0.1 + difference_outer_inner * 0.9)  # top
        right_end_inner = (dist_l_r + conf.wall_length - difference_outer_inner,
                           dist_t_b + conf.wall_length - conf.wall_length * 0.1 - difference_outer_inner * 0.9)  # bottom

        top_start = left_start
        top_end = right_start

        top_start_inner = left_start_inner
        top_end_inner = right_start_inner

        bottom_start = left_end
        bottom_end = right_end

        bottom_start_inner = left_end_inner
        bottom_end_inner = right_end_inner

        walls = {
            'left': [(left_start, left_end), (right_start_inner, right_end_inner)],
            'top': [(top_start, top_end), (bottom_start_inner, bottom_end_inner)],
            'right': [(right_start, right_end), (left_start_inner, left_end_inner)],
            'bottom': [(bottom_start, bottom_end), (top_start_inner, top_end_inner)],
        }

        if conf.position_initialization == "center":
            conf.x = conf.env_width / 2 - 150 - conf.radius
            conf.y = conf.env_height / 2
        elif conf.position_initialization == "corner":
            conf.x = conf.env_width - (conf.env_width - conf.wall_length) / 2 - conf.radius
            conf.y = conf.env_height * 0.8 - (conf.env_height - conf.wall_length) / 2

    return walls


def get_landmarks(conf):
    room_l = (conf.env_width - conf.wall_length) / 2  # distance to frame, left and right
    room_t = (conf.env_height - conf.wall_length) / 2  # distance to frame, top and bottom

    f = 0.17 * conf.wall_length
    positions = [(room_l + 1.2 * f, room_t + 1.5 * f),
                 (room_l + 2.2 * f, room_t + 0.5 * f),
                 (room_l + 2.5 * f, room_t + 1.4 * f),
                 (room_l + 2.5 * f, room_t + 3.4 * f),
                 (room_l + 4.7 * f, room_t + 2.9 * f),
                 (room_l + 5.3 * f, room_t + 3.9 * f),
                 (room_l + 4.8 * f, room_t + 5.1 * f),
                 (room_l + 3.7 * f, room_t + 3.9 * f),
                 (room_l + 2.8 * f, room_t + 2.4 * f),
                 (room_l + 1.8 * f, room_t + 4.9 * f)]

    landmarks = []
    for position in positions:
        landmarks.append(Point(position))

    return landmarks
