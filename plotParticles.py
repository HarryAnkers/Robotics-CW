import time
import sys
import random
from math import *

initX = 500
initY = 120
initTheta = 0

distance10 = 500/4
rotate90 = 90
eSigma = 500/8
fSigma = 10
gSigma = 30

def getNewX(point):
    return point[0] + (distance10 + random.gauss(0, eSigma)) * math.cos(math.radians(point[2]))

def getNewY(point):
    return point[0] + (distance10 + random.gauss(0, eSigma)) * math.sin(math.radians(point[2]))

def getNewThetaStraight(point):
    return point[2] + fSigma
# draws the square
print "are you here?"
line1 = (500, 120, 1000, 120) # (x0, y0, x1, y1)
line2 = (1000, 120, 1000, 620)  # (x0, y0, x1, y1)
line3 = (1000, 620, 500, 620) # (x0, y0, x1, y1)
line4 = (500, 620, 500, 120)  # (x0, y0, x1, y1)

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)
print "drawLine:" + str(line3)
print "drawLine:" + str(line4)

numberOfParticles = 100

def draw_square_plot(interface):
    particles = [(initX, initY, initTheta) for i in range(numberOfParticles)]
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewTheta(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    right90deg(interface)
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    right90deg(interface)
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    right90deg(interface)
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    right90deg(interface)