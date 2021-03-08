def init_walls_coordinates(env_width, env_height, wall_length, room_shape):

    # wall frame distance
    dist_l_r = (env_width - wall_length) / 2  # distance to frame, left and right
    dist_t_b = (env_height - wall_length) / 2  # distance to frame, top and bottom

    if room_shape == "square":

        left_start = (dist_l_r, dist_t_b)
        left_end = (dist_l_r, dist_t_b + wall_length)

        top_start = (dist_l_r, dist_t_b)
        top_end = (dist_l_r + wall_length, dist_t_b)

        right_start = (dist_l_r + wall_length, dist_t_b)
        right_end = (dist_l_r + wall_length, dist_t_b + wall_length)

        bottom_start = (dist_l_r, dist_t_b + wall_length)
        bottom_end = (dist_l_r + wall_length, dist_t_b + wall_length)

        walls = {
            'left': [(left_start, left_end)],
            'top': [(top_start, top_end)],
            'right': [(right_start, right_end)],
            'bottom': [(bottom_start, bottom_end)],
        }

    elif room_shape == "rectangle":

        factor = 0.8

        left_start = (dist_l_r, dist_t_b)
        left_end = (dist_l_r, dist_t_b + wall_length)

        top_start = (dist_l_r, dist_t_b)
        top_end = (dist_l_r + wall_length * factor, dist_t_b)

        right_start = (dist_l_r + wall_length * factor, dist_t_b)
        right_end = (dist_l_r + wall_length * factor, dist_t_b + wall_length)

        bottom_start = (dist_l_r, dist_t_b + wall_length)
        bottom_end = (dist_l_r + wall_length * factor, dist_t_b + wall_length)

        walls = {
            'left': [(left_start, left_end)],
            'top': [(top_start, top_end)],
            'right': [(right_start, right_end)],
            'bottom': [(bottom_start, bottom_end)],
        }

    elif room_shape == "rectangle_double":

        factor = 0.8
        difference_outer_inner = 175

        left_start = (dist_l_r, dist_t_b)
        left_end = (dist_l_r, dist_t_b + wall_length)

        top_start = (dist_l_r, dist_t_b)
        top_end = (dist_l_r + wall_length * factor, dist_t_b)

        right_start = (dist_l_r + wall_length * factor, dist_t_b)
        right_end = (dist_l_r + wall_length * factor, dist_t_b + wall_length)

        bottom_start = (dist_l_r, dist_t_b + wall_length)
        bottom_end = (dist_l_r + wall_length * factor, dist_t_b + wall_length)

        left_start_inner = (dist_l_r + difference_outer_inner, dist_t_b + difference_outer_inner)  # top
        left_end_inner = (dist_l_r + difference_outer_inner, dist_t_b + wall_length - difference_outer_inner)  # bottom

        right_start_inner = (dist_l_r + wall_length * factor - difference_outer_inner, dist_t_b + difference_outer_inner)
        right_end_inner = (dist_l_r + wall_length * factor - difference_outer_inner, dist_t_b + wall_length - difference_outer_inner)

        top_start_inner = (dist_l_r + difference_outer_inner, dist_t_b + difference_outer_inner)
        top_end_inner = (dist_l_r + wall_length * factor - difference_outer_inner, dist_t_b + difference_outer_inner)

        bottom_start_inner = (dist_l_r + difference_outer_inner, dist_t_b + wall_length - difference_outer_inner)
        bottom_end_inner = (dist_l_r + wall_length * factor - difference_outer_inner, dist_t_b + wall_length - difference_outer_inner)

        walls = {
            'left': [(left_start, left_end), (right_start_inner, right_end_inner)],
            'top': [(top_start, top_end), (bottom_start_inner, bottom_end_inner)],
            'right': [(right_start, right_end), (left_start_inner, left_end_inner)],
            'bottom': [(bottom_start, bottom_end), (top_start_inner, top_end_inner)],
        }

    elif room_shape == "trapezoid":

        left_start = (dist_l_r, dist_t_b)  # top
        left_end = (dist_l_r, dist_t_b + wall_length)  # bottom

        right_start = (dist_l_r + wall_length, dist_t_b + wall_length * 0.2)  # top
        right_end = (dist_l_r + wall_length, dist_t_b + wall_length - wall_length * 0.2)  # bottom

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

    if room_shape == "trapezoid_double":

        difference_outer_inner = 200

        left_start = (dist_l_r, dist_t_b)  # top
        left_end = (dist_l_r, dist_t_b + wall_length)  # bottom

        left_start_inner = (dist_l_r + difference_outer_inner, dist_t_b + difference_outer_inner)   # top
        left_end_inner = (dist_l_r + difference_outer_inner, dist_t_b + wall_length - difference_outer_inner)  # bottom

        right_start = (dist_l_r + wall_length, dist_t_b + wall_length * 0.2)  # top
        right_end = (dist_l_r + wall_length, dist_t_b + wall_length - wall_length * 0.2)  # bottom

        right_start_inner = (dist_l_r + wall_length - difference_outer_inner, dist_t_b + wall_length * 0.1 + difference_outer_inner * 0.9)  # top
        right_end_inner = (dist_l_r + wall_length - difference_outer_inner, dist_t_b + wall_length - wall_length * 0.1 - difference_outer_inner * 0.9)  # bottom

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

    return walls