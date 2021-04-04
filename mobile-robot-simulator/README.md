# Mobile Robot Simulator
This project contains a simulator of a mobile robot placed in an environment. It can perform one of two tasks: 
- Navigation
- Localization

In `main.py`, specify which task to perform.

## Navigation with Evolutionary Algorithm
The goal of this task is to navigate through the environment while maximizing the covered / visited area by the robot.

The navigation is optimized by an evolutionary algorithm that performs 
- fitness evaluation based on (i) covered area or (ii) covered-area with collision-free navigation
- truncated rank-based selection (select n best performing individuals)
- generational replacement (replace entire population each generation)
- crossover and mutation (according to crossover and mutation rate)
- termination after stagnation of fitness or max. number of generations

The simulation visualizes the performance (robot navigation) of the best performing individual after each generation. 
Additionally, it is possible to place best individual of last generation in a different environment and test its performance there again.

#### Exemplary Navigation Simulation:

![Navigation](./img/test_navigation.gif)

## Localization with Kalman Filter

The goal of this task is to track the pose (position and orientation) of the robot navigating through the environment. The setup assumes local localization (starting position is known) and known feature correspondence (landmarks' positions are identifiable). 

The simulation visualizes the robots true trajectory (blue) and its believed trajectory (yellow) according to pose tracking with a Kalman Filter. The degree of uncertainty of the robot's belief is depicted by ellipses.

In this simulation, the correction of the Kalman Filter is performed whenever exactly three landmarks are visible to the robot, meaning they are in sensor reach (green dashed line).

Process Noise, Measurement/Sensor noise as well as values for the covariance matrix (uncertainty) are drawn from normal distributions (Gaussian).

#### Exemplary Localization Simulation:

![Localization](./img/localization.gif)