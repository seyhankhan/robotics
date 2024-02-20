import brickpi3
import math
import numpy as np
import random
from time import sleep
from robot import *
from waypoint import *

NUM_PARTICLES = 10

SIGMA_E = 2.0
SIGMA_F = math.radians(1)
SIGMA_G = math.radians(1)
SIGMA_S = 3.0
K = 0.2

WALLS = [
    (0, 0, 0, 168),  # a: O to A
    (0, 168, 84, 168),  # b: A to B
    (84, 126, 84, 210),  # c: C to D
    (84, 210, 168, 210),  # d: D to E
    (168, 210, 168, 84),  # e: E to F
    (168, 84, 210, 84),  # f: F to G
    (210, 84, 210, 0),  # g: G to H
    (210, 0, 0, 0),  # h: H to O
]
WAYPOINTS = [
    (84, 30),
    (180, 30),
    (180, 54),
    (138, 54),
    (138, 168),
    (114, 168),
    (114, 84),
    (84, 84),
    (84, 30),
]

# A Canvas class for drawing a map and particles:
#   - it takes care of a proper scaling and coordinate transformation between
#     the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self, map_size=210):
        self.map_size = map_size  # in cm
        self.canvas_size = 768  # in pixels
        self.margin = 0.05 * map_size
        self.scale = self.canvas_size / (map_size + 2 * self.margin)

    def drawLine(self, line):
        x1 = self._screenX(line[0])
        y1 = self._screenY(line[1])
        x2 = self._screenX(line[2])
        y2 = self._screenY(line[3])
        print(f"drawLine:{(x1, y1, x2, y2)}")

    def drawParticles(self, data):
        print(f"drawParticles:{[(self._screenX(d.x), self._screenY(d.y), d.theta, d.weight) for d in data]}")

    def _screenX(self, x):
        return (x + self.margin) * self.scale

    def _screenY(self, y):
        return (self.map_size + self.margin - y) * self.scale


class Map:
    def __init__(self, walls):
        self.walls = walls

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall)


class Particles:
    def __init__(self, n):
        self.n = n
        # TODO: make this continuous
        self.data = [Particle((0, 0, 0, 1.0 / n)) for _ in range(n)]

    def navigateToWaypoint(self, waypoint):

        for particle in self.data:
            particle.navigateToWaypoint(waypoint)

    def update(self, z, walls):
        # adjust weights with likelihood
        weightSum = 0
        for particle in self.data:
            pzk = particle.likelihood(z, walls)
            print("pzk", pzk)
            particle.weight *= pzk
            weightSum += particle.weight

        # normalization
        for p in self.data:
            p.weight /= weightSum

        # resampling
        cumWeights = np.cumsum([p.weight for p in self.data])
        newData = np.empty_like(self.data)

        for i, randNum in enumerate(np.random.rand(self.n)):
            newData[i] = self.data[np.searchsorted(cumWeights, randNum)]
            newData[i].weight = 1 / self.n

        self.data = newData
        print([(d.x, d.weight) for d in self.data])

    def draw(self):
        canvas.drawParticles(self.data)
    
    def mean_position(self):
        return (
            sum(p.x for p in self.data) / self.n,
            sum(p.y for p in self.data) / self.n,
            sum(p.theta for p in self.data) / self.n,
        )


class Particle:
    def __init__(self, position):
        self.x, self.y, self.theta, self.weight = position

    def position(self):
        return (self.x, self.y, self.theta)

    def navigateToWaypoint(self, waypoint):
        vector = waypoint[0] - self.x, waypoint[1] - self.y
        alpha = math.atan2(vector[1], vector[0])
        self.turn(alpha - self.theta)
        self.forward(min(20, math.hypot(vector[0], vector[1])))

    def forward(self, D):
        e = random.gauss(0, SIGMA_E)
        self.x += (D + e) * math.cos(self.theta)
        self.y += (D + e) * math.sin(self.theta)
        self.theta = math.remainder(self.theta + random.gauss(0, SIGMA_F), math.tau)

    def turn(self, radians):
        g = random.gauss(0, SIGMA_G)
        self.theta = math.remainder(self.theta + radians + g, math.tau)

    def likelihood(self, z, walls):
        # TODO: if beta > SONAR_MAX_ANGLE, skip resample?
        wall, m = self.closest_wall(walls)
        print("z,m",z, m)

        return random.gauss(z - m, SIGMA_S) + K

    def closest_wall(self, walls):
        closestWall, smallestDistance = None, float("inf")
        print('theta x, y', math.degrees(self.theta), self.x, self.y)

        for wall in walls:
            (ax, ay, bx, by) = wall

            m = ((by - ay) * (ax - self.x) - (bx - ax) * (ay - self.y)) / (
                (by - ay) * math.cos(self.theta) - (bx - ax) * math.sin(self.theta)
            )
            if m < 0:
                continue
            mx, my = (self.x + m * math.cos(self.theta)), (self.y + m * math.sin(self.theta))

            print("wall coords", wall)
            print("m , mx,my", m, mx,my)
            if ((min(ax, bx) - EPSILON) < mx < (max(ax, bx) + EPSILON)) and ((min(ay, by) - EPSILON) < my < (max(ay, by) + EPSILON)) and (m < smallestDistance):
                
                closestWall, smallestDistance = wall, m

        return closestWall, smallestDistance


if __name__ == "__main__":
    try:
        canvas = Canvas()
        mymap = Map(WALLS)
        mymap.draw()

        particles = Particles(NUM_PARTICLES)
        particles.draw()

        Robot.setupSonar()
        Robot.setPosition(WAYPOINTS[0] + (0,))

        for waypoint in WAYPOINTS[1:]:
            robotVector = (waypoint[0] - Robot.x, waypoint[1] - Robot.y)

            robotAlpha = math.atan2(robotVector[1], robotVector[0])
            robotBeta = math.remainder(robotAlpha - Robot.theta, math.tau)
            Robot.turn(robotBeta)
            for particle in particles.data:
                particle.turn(robotBeta)

            remainingDistance = math.hypot(robotVector[0], robotVector[1])
            while remainingDistance > 0:
                Robot.forward(min(20, remainingDistance))
                for particle in particles.data:
                    particle.forward(min(20, remainingDistance))
                remainingDistance -= 20

                z = Robot.readSonar()
                particles.update(z, mymap.walls)
                Robot.setPosition(particles.mean_position())
                particles.draw()
                sleep(1)

            # Robot.navigateToWaypoint(waypoint)
            # particles.navigateToWaypoint(waypoint)

    except KeyboardInterrupt:
        BP.reset_all()
    finally:
        BP.reset_all()
