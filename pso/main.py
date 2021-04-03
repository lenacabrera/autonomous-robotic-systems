import pso

if __name__ == '__main__':
    pso.optimize(n_particles=20,                   # population size
                 n_iterations=130,                 # number of iterations / updates
                 benchmark_function='rastrigin',   # rastrigin, rosenbrock
                 a=0.9,                            # learning constant: conservative = "continue with current velocity"
                 b=2,                              # learning constant: brave = "direction to local best location"
                 c=2,                              # learning constant: swarm = "direction to global best location"
                 r_max=1,                          # random factor maximum
                 delta_t=1,                        # euler integration
                 frame_range=[-5, 5],              # size of (squared) frame
                 random_init_v=False,              # random initialization of velocity, if False init. with 0
                 v_max=5,                          # maximum velocity
                 oof_strategy=4                    # out of frame strategy:
                 )                                  # 0: old position,
                                                    # 1: change direction,
                                                    # 2: new random factors (r1, r2),
                                                    # 3: only small step in new direction,
                                                    # 4: old coordinate