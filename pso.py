from particle import Particle
import numpy
import random
from pygame import *
import benchmark_functions


def pso(n_particles, n_iterations, benchmark_function, a, b, c, r_max, delta_t, v_max, frame_range, random_init_v,
        oof_strategy):
    if random_init_v:
        # set velocity to random value
        init_velocity = numpy.array([random.uniform(-v_max, v_max), random.uniform(-v_max, v_max)])
    else:
        # set velocity to zero
        init_velocity = numpy.array([0, 0])

    # initialize all particles
    particles = []
    gbest = numpy.array([-1, -1])
    gbest_fitness = -1

    for i_particle in range(n_particles):
        x_rand = random.uniform(frame_range[0], frame_range[1])
        y_rand = random.uniform(frame_range[0], frame_range[1])
        init_position = numpy.array([x_rand, y_rand])
        print(init_position)
        particle = Particle(init_velocity, init_position)

        if benchmark_function == 'rosenbrock':
            p_fitness = benchmark_functions.rosenbrock(particle.p)

        if benchmark_function == 'rastrigin':
            p_fitness = benchmark_functions.rastrigin(particle.p)

        if p_fitness > gbest_fitness:
            gbest_fitness = p_fitness
            gbest = particle.p

        particles.append(particle)

    # TODO look at other
    screen = display.set_mode((2 * frame_range[1], 2 * frame_range[1]))
    # Fill screen white
    screen.fill((255, 255, 255))

    def waitFor(waitTime):  # waitTime in milliseconds
        screenCopy = screen.copy()
        waitCount = 0
        while waitCount < waitTime:
            clock = time.Clock()
            dt = clock.tick(60)  # 60 is your FPS here
            waitCount += dt
            event.pump()  # Tells pygame to handle it's event, instead of pygame.event.get()
            screen.blit(screenCopy, (0, 0))
            display.flip()

    # Game loop: idea from: https://www.petercollingridge.co.uk/tutorials/pygame-physics-simulation/
    running = True
    while running:
        # Properly quit (pygame will crash without this)
        for e in event.get():
            if e.type == QUIT:
                running = False

        # update
        screen.fill((255, 255, 255))
        for particle in particles:
            particle.update(gbest, a, b, c, r_max, delta_t, v_max, frame_range, oof_strategy)
            particle.evaluate(benchmark_function)
            particle.display(screen, frame_range)

            if particle.pbest_fitness > gbest_fitness:
                gbest_fitness = particle.pbest_fitness
                gbest = particle.pbest

        n_iterations = n_iterations - 1
        print(n_iterations)

        if n_iterations == 1:
            running = False
            for particle in particles:
                print(particle.p)

        # decrease a
        decrease_amount = (0.9 - 0.4) / n_iterations
        a -= decrease_amount

        waitFor(500)
        display.flip()
    quit()


if __name__ == '__main__':
    pso(n_particles=20,
        n_iterations=100,
        benchmark_function='rastrigin',
        a=0.9,
        b=2,
        c=2,
        r_max=1,
        delta_t=1,
        v_max=30,
        frame_range=[-100, 100],
        random_init_v=False,
        # "out of screen strategy"
        # 0=old position,
        # 1=change direction,
        # 2=new random (r1, r2),
        # 3=only small step in new direction,
        # 4=old coordinate
        oof_strategy=3
        )

    exit()