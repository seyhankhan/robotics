import brickpi3
import math
import numpy as np
import random
from time import sleep
from robot import *
from waypoint import *

NUM_PARTICLES = 120

SIGMA_E = 1.0
SIGMA_F = math.radians(1.0)
SIGMA_G = math.radians(3.0)
SIGMA_S = 1.0
K = 0.03

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
    
    def _screenWaypoints(self):
        return [(self._screenX(x), self._screenY(y), 0) for (x,y) in WAYPOINTS]

    def drawParticles(self, data):
        print(
            f"drawParticles:{[(self._screenX(d.x), self._screenY(d.y), d.theta) for d in data] + self._screenWaypoints()}"
        )

    def _screenX(self, x):
        return (x + self.margin) * self.scale

    def _screenY(self, y):
        return (self.map_size + self.margin - y) * self.scale


class Particles:
    def __init__(self, n, position):
        self.n = n
        self.data = [Particle(position + (1.0 / n,)) for _ in range(n)]

    def __str__(self):
        output = f"{self.n} Particles:\n"
        for p in self.data:
            output += str(p)
        return output

    def update(self, z, walls):
        # adjust weights with likelihood
        weightSum = 0
        for p in range(self.n):
            pzk = self.data[p].likelihood(z, walls)
            self.data[p].weight *= pzk
            weightSum += self.data[p].weight

        if weightSum == 0:
            for p in range(self.n):
                self.data[p].weight = 1.0 / self.n
                return False
        # normalization
        for p in range(self.n):
            self.data[p].weight /= weightSum

        # resampling
        cumWeights = np.cumsum([p.weight for p in self.data]) / sum(
            [p.weight for p in self.data]
        )
        newData = []

        for i, randNum in enumerate(np.random.rand(self.n)):
            particle = self.data[np.searchsorted(cumWeights, randNum)]
            newData.append(Particle((particle.x, particle.y, particle.theta, 1.0 / self.n)))

        self.data = newData
        return True

    def mean_theta(self):
        xs = ys = 0
        for particle in self.data:
            xs += math.cos(particle.theta)
            ys += math.sin(particle.theta)
        xs /= self.n
        ys /= self.n
        return math.remainder(math.atan2(ys, xs), math.tau)

    def draw(self):
        canvas.drawParticles(self.data)

    def mean_position(self):
        return (
            sum(p.x for p in self.data) / self.n,
            sum(p.y for p in self.data) / self.n,
            self.mean_theta(),
        )


class Particle:
    def __init__(self, position):
        self.x, self.y, self.theta, self.weight = position

    def __str__(self):
        return f"\t{self.x:.1f}\t{self.y:.1f}\t{math.degrees(self.theta):.1f}\t{self.weight:.5f}\n"

    def forward(self, D):
        e = random.gauss(0, SIGMA_E)
        self.x += (D + e) * math.cos(self.theta)
        self.y += (D + e) * math.sin(self.theta)
        self.theta = math.remainder(random.gauss(self.theta, SIGMA_F), math.tau)

    def turn(self, radians):
        g = random.gauss(0, SIGMA_G)
        self.theta = math.remainder(self.theta + radians + g, math.tau)

    def likelihood(self, z, walls):
        wall, m = self.closest_wall(walls)
        if m == float("inf"):
            return 0
        return math.exp(-((z - m) ** 2) / (2 * SIGMA_S**2)) + K

    def closest_wall(self, walls):
        closestWall, smallestDistance = None, float("inf")

        for wall in walls:
            (ax, ay, bx, by) = wall

            m = ((by - ay) * (ax - self.x) - (bx - ax) * (ay - self.y)) / (
                (by - ay) * math.cos(self.theta) - (bx - ax) * math.sin(self.theta)
            )
            # if wall is behind: skip
            if m < 0:
                continue

            mx = self.x + m * math.cos(self.theta)
            my = self.y + m * math.sin(self.theta)

            if (
                ((min(ax, bx) - EPSILON) < mx < (max(ax, bx) + EPSILON))
                and ((min(ay, by) - EPSILON) < my < (max(ay, by) + EPSILON))
                and (m < smallestDistance)
            ):
                particleVector = np.array([math.cos(self.theta), np.sin(self.theta)])
                normalToWall = np.array([ay - by, bx - ax])

                incidenceTheta = np.arccos(np.dot(particleVector, normalToWall) / (
                    np.linalg.norm(particleVector) * np.linalg.norm(normalToWall)
                ))
                if incidenceTheta > SONAR_MAX_ANGLE:
                    print(f"Sonar can't reliably read at {math.degrees(incidenceTheta):.1f} degrees to the wall {wall}")
                    continue
                closestWall, smallestDistance = wall, m

        return closestWall, smallestDistance


if __name__ == "__main__":
    try:
        canvas = Canvas()
        for wall in WALLS:
            canvas.drawLine(wall)

        particles = Particles(NUM_PARTICLES, WAYPOINTS[0] + (0.0,))
        particles.draw()

        Robot.setupSonar()
        Robot.setPosition(WAYPOINTS[0] + (0,))

        for waypoint in WAYPOINTS[1:]:
            print("Waypoint:", waypoint)
            while True:                
                # Success if 3cm from waypoint
                robotVector = (waypoint[0] - Robot.x, waypoint[1] - Robot.y)
                if math.hypot(robotVector[0], robotVector[1]) < 3:
                    break

                # TURN
                robotAlpha = math.atan2(robotVector[1], robotVector[0])
                robotBeta = math.remainder(robotAlpha - Robot.theta, math.tau)
                Robot.turn(robotBeta)
                for particle in particles.data:
                    particle.turn(robotBeta)
                
                # FORWARD
                remainingDistance = math.hypot(robotVector[0], robotVector[1])
                oldPos = (Robot.x, Robot.y)
                Robot.forward(min(20, remainingDistance))
                for p in range(particles.n):
                    particles.data[p].forward(min(20, remainingDistance))

                # ADJUSTMENT
                z = Robot.readSonar()
                success = particles.update(z, WALLS)
                if success:
                    Robot.setPosition(particles.mean_position())
                else:
                    print("Unreliable sonar update")

                # DRAW PATH
                canvas.drawLine(oldPos + (Robot.x, Robot.y))
                particles.draw()
                sleep(2)
                

    except KeyboardInterrupt:
        BP.reset_all()
    finally:
        BP.reset_all()
