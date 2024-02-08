from time import sleep  # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import sys
import random
import math

EPSILON = 10

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

ports = [BP.PORT_B, BP.PORT_C]
        
class Particle:
    def __init__(self, pos):
        self.x = pos[0]
        self.y = pos[1]
        self.theta = pos[2]

    def changeDistance(self, D):
        sigmaE = 2
        sigmaF = math.pi / 90
        e = random.gauss(0, sigmaE)
        f = random.gauss(0, sigmaF)
        self.x = round(self.x + (D + e) * math.cos(self.theta), 2)
        self.y = round(self.y + (D + e) * math.sin(self.theta), 2)
        self.theta = round(self.theta + f, 2)

    def changeAngle(self, alpha):
        sigmaG = (math.pi / 90)
        g = random.gauss(0, sigmaG)
        self.theta = self.theta + alpha + g

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta})"
        
def drawParticles(particles):
    ps= list(map(lambda p: (p.x, p.y, p.theta), particles))
    print("drawParticles:" + str(ps))

try:
    
    pos = (400, 500, 0)
    NUM_PARTICLES = 100
    particles = [Particle(pos) for _ in range(NUM_PARTICLES)]
    drawParticles(particles)
  
    
    for _ in range(4):
        for _ in range(4):
            (x, y, theta) = pos
            
            # move frwd
            sleep(2)
            for p in particles:
                p.changeDistance(100)
                
            xs = 0
            ys = 0            
            for p in particles:            
                xs += p.x
                ys += p.y
                
            xs /= NUM_PARTICLES
            ys /= NUM_PARTICLES
            
            
            # print
            print(f"drawLine:{(x, y, xs, ys)}")
            drawParticles(particles)

            pos = (xs, ys, theta)
            
            
        [p.changeAngle(-math.pi / 2) for p in particles]
        (x, y, _) = pos
        theta = sum(map(lambda p: p.theta, particles)) / NUM_PARTICLES
        pos = (x, y, theta)
            
    # print(f"drawLine:{(500, 400, 600, 400)}")
    # for particle in particles:
    #     print(f"drawParticles:" + str((particle.x, particle.y, particle.theta)))
    #     particle.changeDistance(100)
    #     print(f"drawParticles:" + str((particle.x, particle.y, particle.theta)))
    #     print("NEW:", particle.x)
        

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()  # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
        
finally:
    BP.reset_all()  
