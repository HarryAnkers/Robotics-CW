import brickpi
import time
import sys
import random
import math
import numpy 

interface=brickpi.Interface()
interface.initialize()

motors = [1,2]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

leftParam = interface.MotorAngleControllerParameters()
rightParam = interface.MotorAngleControllerParameters()

leftParam.maxRotationAcceleration = 6.0
leftParam.maxRotationSpeed = 12.0
leftParam.feedForwardGain = 255/20.0
leftParam.minPWM = 18.0
leftParam.pidParameters.minOutput = -255
leftParam.pidParameters.maxOutput = 255
leftParam.pidParameters.k_p = 462.0
leftParam.pidParameters.k_i = 870.0
leftParam.pidParameters.K_d = 25.0

rightParam.maxRotationAcceleration = 6.0
rightParam.maxRotationSpeed = 12.0
rightParam.feedForwardGain = 255/20.0
rightParam.minPWM = 18.0
rightParam.pidParameters.minOutput = -255
rightParam.pidParameters.maxOutput = 255
rightParam.pidParameters.k_p = 462.0
rightParam.pidParameters.k_i = 800.0
rightParam.pidParameters.K_d = 23.0

interface.setMotorAngleControllerParameters(motors[0],leftParam)
interface.setMotorAngleControllerParameters(motors[1],rightParam)


rotation_angle = 3.3
drive_angle = 11.85
drive_angle_10 = 2.92

def forward(interface):
    interface.increaseMotorAngleReferences(motors, [drive_angle, drive_angle])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        #if motorAngles :
        #    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)

def forward_10(interface):
    interface.increaseMotorAngleReferences(motors, [drive_angle_10, drive_angle_10])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        #if motorAngles :
        #    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)

def backward(interface):
    interface.increaseMotorAngleReferences(motors, [-drive_angle, -drive_angle])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        if motorAngles :
            print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)

def backward_10(interface):
    interface.increaseMotorAngleReferences(motors, [-drive_angle_10, -drive_angle_10])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        if motorAngles :
            print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)

def right90deg(interface):
    interface.increaseMotorAngleReferences(motors, [-rotation_angle, rotation_angle])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        #if motorAngles :
        #    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)

def left90deg(interface):
    interface.increaseMotorAngleReferences(motors, [rotation_angle, -rotation_angle])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        if motorAngles :
            print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)

def draw_square(interface):
    forward(interface)
    right90deg(interface)
    forward(interface)
    right90deg(interface)
    forward(interface)
    right90deg(interface)
    forward(interface)
    right90deg(interface)

def draw_square_10(interface):
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
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    forward_10(interface)
    right90deg(interface)
    
    
initX = 200
initY = 120
initTheta = 0

distance10 = 500/4
rotate90 = 90
eSigma = 500 * 0.0075
fSigma = 0.4
gSigma = 0.8

def getNewX(point):
    return point[0] + (distance10 + random.gauss(0, eSigma)) * math.cos(math.radians(point[2]))

def getNewY(point):
    return point[1] + (distance10 + random.gauss(0, eSigma)) * math.sin(math.radians(point[2]))

def getNewThetaStraight(point):
    return point[2] + random.gauss(0, fSigma)

def getNewThetaRotate(point):
    return point[2] + rotate90 + random.gauss(0, gSigma)
# draws the square
line1 = (200, 120, 700, 120) # (x0, y0, x1, y1)
line2 = (700, 120, 700, 620)  # (x0, y0, x1, y1)
line3 = (700, 620, 200, 620) # (x0, y0, x1, y1)
line4 = (200, 620, 200, 120)  # (x0, y0, x1, y1)

print "drawLine:" + str(line1)
print "drawLine:" + str(line2)
print "drawLine:" + str(line3)
print "drawLine:" + str(line4)

numberOfParticles = 100

def draw_square_plot(interface):
    particles = [(initX, initY, initTheta) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    
    right90deg(interface)
    particles = [(particles[i][0], particles[i][1], getNewThetaRotate(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    
    right90deg(interface)
    particles = [(particles[i][0], particles[i][1], getNewThetaRotate(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    
    right90deg(interface)
    particles = [(particles[i][0], particles[i][1], getNewThetaRotate(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    forward_10(interface)
    particles = [(getNewX(particles[i]), getNewY(particles[i]), getNewThetaStraight(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    time.sleep(0.1)
    
    right90deg(interface)
    particles = [(particles[i][0], particles[i][1], getNewThetaRotate(particles[i])) for i in range(numberOfParticles)]
    print "drawParticles:" + str(particles)
    numpy_arr = numpy.array([particle[0] for particle in particles])
    print(numpy.std(numpy_arr))
    time.sleep(0.1)


draw_square_plot(interface)


interface.terminate()
