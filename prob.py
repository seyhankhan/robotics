import brickpi3
import math
import random
from time import sleep
from robot import *

NUM_PARTICLES = 100

SQUARE_LENGTH = 40
STEP_LENGTH = 10

PX_PER_CM = 10
X_AXIS = 400
Y_AXIS = 500

SIGMA_E = 2.0
SIGMA_F = math.radians(1)
SIGMA_G = math.radians(1)


class ParticleDistribution:
    def __init__(self, numParticles):
        self.particles = [Particle() for _ in range(numParticles)]

    def forward(self, distanceCM):
        [p.move(distanceCM) for p in particles]

    def turn(self, alpha):
        [p.turn(alpha) for p in particles]

    def get_mean_position(self):
        return (
            sum(p.x for p in self.particles) / NUM_PARTICLES,
            sum(p.y for p in self.particles) / NUM_PARTICLES,
            sum(p.theta for p in self.particles) / NUM_PARTICLES,
        )

    def draw(self):
        print(f"drawParticles:{[p.screen_position() for p in self.particles]}")


class Particle:
    def __init__(self):
        self.x, self.y, self.theta = (0, 0, 0)

    def move(self, D):
        e = random.gauss(0, SIGMA_E)
        f = random.gauss(0, SIGMA_F)
        self.x = self.x + (D + e) * math.cos(self.theta)
        self.y = self.y + (D + e) * math.sin(self.theta)
        self.theta = math.remainder(self.theta + f, math.tau)

    def turn(self, alpha):
        g = random.gauss(0, SIGMA_G)
        self.theta = math.remainder(self.theta + alpha + g, math.tau)

    def screen_position(self):
        return (self.x * PX_PER_CM + X_AXIS, self.y * PX_PER_CM + Y_AXIS, self.theta)


if __name__ == "__main__":
    try:
        robotPosition = (0, 0, 0)

        particles = ParticleDistribution(NUM_PARTICLES)
        particles.draw()

        for _ in range(4):
            for _ in range(SQUARE_LENGTH / STEP_LENGTH):
                Robot.forward(STEP_LENGTH)

                particles.forward(STEP_LENGTH)
                meanPosition = particles.get_mean_position()
                print(f"drawLine:{robotPosition[:2] + meanPosition[:2]}")
                robotPosition = meanPosition
                particles.draw()
                sleep(2)

            Robot.turn(math.pi / 2)
            particles.turn(-math.pi / 2)
            sleep(1)

    except KeyboardInterrupt:
        pass
    BP.reset_all()
