Evolutionary Algorithm for benchmark functions "rosenbrock" and "rastrigin"
(This program was jointly programmed by Kathrin Hartmann and Lena Cabrera)

Use of program:

1. Configure the simulation setup through parameters in the function "evolutionary_algorithm" in the file "main.py":
- n_individuals             size of population
- n_iterations              maximum number of generations
- benchmark_function        either 'rastrigin' or 'rosenbrock'
- frame_range               range of search space (visualization)
- n_best_percentage         x percent of best performing individuals in one generation (selection)
- crossover_percentage      crossover on x percent of best performing individuals in one generation (crossover)
- mutation_percentage       with x percent change performing mutation on a randomly picked individual in one generation (mutation)
- termination_threshold     threshold for termination in case of x same or worse performance than previous generation in a row

2. Run the main function in main.py




TODO
- crossover anpassen (position where to cut, randomly selected) <- debug Lena
- Fitness function <- punish collision
- (Grid + dust (simulation))
- different rooms <- Lena (e.g train in 2 rooms, test 2 rooms)

- title generations in animation
- plots <- Kathrin
-  how to save animation of different generations without running simulation live (for video)