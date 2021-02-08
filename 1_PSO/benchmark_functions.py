import math


def rosenbrock(p):
    x = p[0]
    y = p[1]
    a = 0
    b = 100
    fitness = pow(a - x, 2) + b * pow(y - pow(x, 2), 2)
    return fitness


def rastrigin(p):
    x = p[0]
    y = p[1]
    fitness = 10 * 2 + (pow(x, 2) - 10 * math.cos(2 * math.pi * x)) + (pow(y, 2) - 10 * math.cos(2 * math.pi * y))
    return fitness
