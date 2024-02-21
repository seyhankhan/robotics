from mcl import *

particles = Particles(10)


def forward(n=20):
    for p in range(10):
        particles.data[p].forward(n)


def update(z):
    particles.update(z, WALLS)


def show():
    print(particles)


def turn(n):
    for p in range(10):
        particles.data[p].turn(n)
