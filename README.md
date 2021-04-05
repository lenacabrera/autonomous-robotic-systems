# Autonomous Robotic Systems

This project contains several techniques and algorithms used in autonomous robotic systems, including swarm intelligence, evolutionary algorithms, and localization.

## Mobile Robot Simulator
The simulator places a mobile robot in an environment where one of two tasks is performed: 
- Navigation
- Localization

In `mobile-robot-simulator/src/main.py`, specify which task to perform.

### Navigation with Evolutionary Algorithm
The goal of this task is to navigate the robot through the environment while maximizing the covered / visited area by the robot.

The navigation is optimized by an evolutionary algorithm that performs 
- fitness evaluation based on (i) covered area or (ii) covered-area with collision-free navigation
- truncated rank-based selection (select n best performing individuals)
- generational replacement (replace entire population each generation)
- crossover and mutation (according to crossover and mutation rate)
- termination after stagnation of fitness or max. number of generations

The simulation can visualize the performance (robot navigation) of the best performing individual after each generation. 
Additionally, it is possible to place the best individual of the last generation in a different environment and test its performance there again.

### Localization with Kalman Filter

The goal of this task is to track the pose (position and orientation) of the robot (manually) navigating through the environment. The setup assumes local localization (starting position is known) and known feature correspondence (landmarks' positions are identifiable). 

The simulation visualizes the robots true trajectory (blue solid line) and its believed trajectory (yellow solid line) according to pose tracking with a Kalman Filter. The degree of uncertainty of the robot's belief is depicted by ellipses (planar Gaussians).

In this simulation, the correction of the Kalman Filter is performed whenever exactly three landmarks are visible to the robot, meaning they are in sensor reach (green dashed line).

Process noise, measurement (sensor) noise and values for uncertainty (covariance matrix) are drawn from normal distributions.

| Exemplary Navigation Simulation                                 | Exemplary Localization Simulation                              |
| --------------------------------------------------------------- |:--------------------------------------------------------------:|
| ![Navigation](./mobile-robot-simulator/img/test_navigation.gif) | ![Localization](./mobile-robot-simulator/img/localization.gif) |

## Particle Swarm Optimization (PSO)

Optimization technique that aims to find the optimal value (e.g. global minima) in a fitness landscape through emergent collective behavior and intelligence in a decentralized self-organizing systems.

Make use of this by providing a function to be optimized and running the optimization.