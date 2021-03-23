Self-localization wit Kalman Filter
(This program was jointly programmed by Kathrin Hartmann and Lena Cabrera)

Use of program:

1. Configure the settings of the Kalman Filter (e.g. range of normal distribution from which process noise and sensor noise are drawn) in kalman_filter.py file (init)
2. Run the main function in main.py
3. Navigate the robot through the environment the following keys
    W -> increment velocity
    S -> decrement velocity
    D -> increment rotation
    A -> decrement rotation
    X -> stop (set velocity to zero and keep current orientation)
