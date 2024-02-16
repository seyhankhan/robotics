import brickpi3
from time import sleep
import random
import math
import numpy as np
from robot import *
from waypoint import *

SIGMA_S = 3.0
K = 0.2

RELIABILITY_CEILING = 140

SIGMA_E = 2.0
SIGMA_F = math.pi / 180.0
SIGMA_G = math.pi / 180.0

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80, 3) + 70 * (math.sin(t))  # in cm


def calcY():
    return random.gauss(70, 3) + 60 * (math.sin(2 * t))  # in cm


def calcW():
    return random.random()


def calcTheta():
    return random.randint(0, 360)


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
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print("drawLine:" + str((x1, y1, x2, y2)))

    def drawParticles(self, data):
        display = [(self.__screenX(d.x), self.__screenY(d.y)) + (d.theta,d.weight) for d in data]
        print("drawParticles:" + str(display))

    def __screenX(self, x):
        return (x + self.margin) * self.scale

    def __screenY(self, y):
        return (self.map_size + self.margin - y) * self.scale


# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = []

    def add_wall(self, wall):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall)

    def closest_wall(self, x, y, theta):
        sigmaWall, smallestDistance = None, float("inf")

        for wall in self.walls:
            (ax, ay, bx, by) = wall

            m = ((by - ay) * (ax - x) - (bx - ax) * (ay - y)) / (
                (by - ay) * math.cos(theta) - (bx - ax) * math.sin(theta)
            )
            if m < 0:
                continue
            mx, my = x + m * math.sin(theta), y + m * math.cos(theta)
            if ax < mx and mx < bx and ay < my and my < by and m < smallestDistance:
                smallestDistance, sigmaWall = m, wall

        return sigmaWall, smallestDistance


# Simple Particles set
class Particles:
    def __init__(self, n):
        self.n = n
        # make this continous
        self.data = [Particle((0, 0, 0, 1 / n)) for _ in range(n)]

    def navigateToWaypoint(self, waypoint):
        for particle in self.data:
            particle.navigateToWaypoint(waypoint)

    def update(self, z, mymap):
        weightSum = 0
        for i in range(self.n):
            (x, y, theta, w) = self.data[i]
            pzk = calculate_likelihood(x, y, theta, z, mymap)
            self.data[i].weight *= pzk
            weightSum += w * pzk

        # normalization
        for i in range(self.n):
            self.data[i].weight /= weightSum

        # resampling
        cumWeights = list(np.cumsum(p.weight for p in self.data))
        newData = []
        for i in range(self.n):
            rand = random.random()
            for j in range(self.n):
                if rand < cumWeights[j]:
                    oldParticle = self.data[j]
                    oldParticle.weight = 1 / self.n
                    newData.append(oldParticle)
                    break

        self.data = newData

    def draw(self):
        canvas.drawParticles(self.data)

class Particle:
    def __init__(self, position):
        self.x, self.y, self.theta, self.weight = position

    def navigateToWaypoint(self, waypoint):
        position = self.x, self.y, self.theta, self.weight
        self.turn(getBeta(waypoint, position))
        self.forward(getDistance(waypoint, position))

    def forward(self, D):
        e = random.gauss(0, SIGMA_E)
        f = random.gauss(0, SIGMA_F)
        self.x = self.x + (D + e) * math.cos(self.theta)
        self.y = self.y + (D + e) * math.sin(self.theta)
        self.theta = self.theta + f

    def turn(self, alpha):
        g = random.gauss(0, SIGMA_G)
        self.theta = self.theta + alpha + g


def calculate_likelihood(x, y, theta, z, mymap):
    # if beta > 34 * math.pi / 180:
    #     skip the whole update
    wall, m = mymap.closest_wall(x, y, theta * math.pi / 180)
    return random.gauss(z - m, SIGMA_S) + K


canvas = Canvas()  # global canvas we are going to draw on

mymap = Map()
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0, 0, 0, 168))  # a
mymap.add_wall((0, 168, 84, 168))  # b
mymap.add_wall((84, 126, 84, 210))  # c
mymap.add_wall((84, 210, 168, 210))  # d
mymap.add_wall((168, 210, 168, 84))  # e
mymap.add_wall((168, 84, 210, 84))  # f
mymap.add_wall((210, 84, 210, 0))  # g
mymap.add_wall((210, 0, 0, 0))  # h
mymap.draw()

waypoints = [
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

particles = Particles(10)
robotPosition = waypoints[0] + (0,)
print(4)

try:
    BP.set_sensor_type(SENSOR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)
    particles.draw()

    for waypoint in waypoints[1:]:
        robotPosition = navigateToWaypoint(waypoint, robotPosition)
        particles.navigateToWaypoint(waypoint)
        while True:
            try:
                z = BP.get_sensor(SENSOR_PORT)
                print(z)
                if z < RELIABILITY_CEILING:
                    break
            except brickpi3.SensorError as error:
                print(error)
            sleep(0.2)

        particles.update(z, mymap)
        particles.draw()
        # t += 0.5
        sleep(1)

except KeyboardInterrupt:
    BP.reset_all()
BP.reset_all()