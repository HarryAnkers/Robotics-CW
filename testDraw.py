import time
import sys
import random

c = 0;
def getRandomX():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomY():
    return random.randint((c%10)*50, (c%10 + 1)*50)

def getRandomTheta(): 
    return random.randint(0, 360)

numberOfParticles = 100

line1 = (500, 120, 1000, 120) # (x0, y0, x1, y1)
line2 = (1000, 120, 1000, 620)  # (x0, y0, x1, y1)
line3 = (1000, 620, 500, 620) # (x0, y0, x1, y1)
line4 = (500, 620, 500, 120)  # (x0, y0, x1, y1)

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)
print "drawLine:" + str(line3)
print "drawLine:" + str(line4)

while True:
    # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
    particles = [(getRandomX(), getRandomY(), getRandomTheta()) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    
    c += 1;
    time.sleep(0.25)
