from particle import Particle
import numpy
import random
from pygame import *
import benchmark_functions

def pso(n_particles, n_iterations, benchmark_function):

    init_velocity = numpy.array([0, 0]) #zeros(2)  # angle and distance  TODO

    # initialize all particles
    particles = []
    gbest = numpy.array([-1, -1])
    gbest_fitness = -1

    for i_particle in range(n_particles):
        x_rand = random.uniform(0, 200)  # TODO
        y_rand = random.uniform(0, 200)  # TODO
        init_position = numpy.array([x_rand, y_rand])
        particle = Particle(init_velocity, init_position)

        if benchmark_function == 'rosenbrock':
            p_fitness = benchmark_functions.rosenbrock(particle.p)

        if benchmark_function == 'rastrigin':
            p_fitness = benchmark_functions.rastrigin(particle.p)

        if p_fitness > gbest_fitness:
            gbest_fitness = p_fitness
            gbest = particle.p

        particles.append(particle)

    a = 0.9
    b = 2
    c = 2
    delta_t = 1
    v_max = 30

    screen = display.set_mode((200, 200))
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
            particle.update(gbest, a, b, c, delta_t, v_max)
            particle.evaluate(benchmark_function)
            particle.display(screen)

            if particle.pbest_fitness > gbest_fitness:
                gbest_fitness = particle.pbest_fitness
                gbest = particle.pbest

        n_iterations = n_iterations-1

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


    pso(20, 100, 'rastrigin')
