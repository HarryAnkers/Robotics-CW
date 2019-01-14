import brickpi
import time
import sys
import random
import math
import numpy as np

# interface=brickpi.Interface()
# interface.initialize()
#
# motors = [1,2]
#
# interface.motorEnable(motors[0])
# interface.motorEnable(motors[1])
#
# leftParam = interface.MotorAngleControllerParameters()
# rightParam = interface.MotorAngleControllerParameters()
#
# leftParam.maxRotationAcceleration = 6.0
# leftParam.maxRotationSpeed = 12.0
# leftParam.feedForwardGain = 255/20.0
# leftParam.minPWM = 18.0
# leftParam.pidParameters.minOutput = -255
# leftParam.pidParameters.maxOutput = 255
# leftParam.pidParameters.k_p = 462.0
# leftParam.pidParameters.k_i = 870.0
# leftParam.pidParameters.K_d = 25.0
#
# rightParam.maxRotationAcceleration = 6.0
# rightParam.maxRotationSpeed = 12.0
# rightParam.feedForwardGain = 255/20.0
# rightParam.minPWM = 18.0
# rightParam.pidParameters.minOutput = -255
# rightParam.pidParameters.maxOutput = 255
# rightParam.pidParameters.k_p = 462.0
# rightParam.pidParameters.k_i = 800.0
# rightParam.pidParameters.K_d = 23.0
#
# interface.setMotorAngleControllerParameters(motors[0],leftParam)
# interface.setMotorAngleControllerParameters(motors[1],rightParam)
#
# port = 0 # port which ultrasoic sensor is plugged in to
#
# interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

#initX = 0
#initY = 0
#initA = 0

initX = 84
initY = 30
initA = 0
rotation_angle = 2.12
drive_angle = 0.28
drive_angle_10 = 2.92

numberOfParticles = 100

global interface

def set_interface(input):
    interface = input

# walls = [((0,0), (0,168)), ((0,168), (84, 168)), ((84, 168), (84, 126)), ((84, 126), (84, 210)), ((84, 210), (168, 210)), ((168, 210), (168, 84)), ((168, 84), (210, 84)), ((210, 84), (210, 0)), ((210, 0), (0, 0))]
#
# for wall in walls:
#     print "drawLine:" + str(( 50 + wall[0][0] * 3,50 + wall[0][1] * 3, 50 +wall[1][0] * 3, 50 +wall[1][1] * 3))

######## navigate to waypoint #########

robotMeanX, robotMeanY, robotMeanA = 84, 30, 0 #initial position
#robotMeanX, robotMeanY, robotMeanA = 0, 0, 0 #initial position

particles = [(initX, initY, initA) for i in range(numberOfParticles)]

def set_starting_point(x, y, a):
    global initX, initY, initA, robotMeanA, robotMeanX, robotMeanY, particles
    initX = x
    initY = y
    initA = a
    particles = [(initX, initY, initA) for i in range(numberOfParticles)]
    robotMeanX = x
    robotMeanY = y
    robotMeanA = a

def navigateToWayPointInput(interface):
    global robotMeanX, robotMeanY, robotMeanA
    wayPointX, wayPointY = 0, 0
    while True:
        wayPointX = float(input("Enter wayPointX in metres: ")) * 100
        wayPointY = float(input("Enter wayPointY in metres: ")) * 100

        xDist = wayPointX - robotMeanX #- wayPointX
        yDist = wayPointY - robotMeanY #- wayPointY
        angle = math.atan2(yDist, xDist) - robotMeanA
        #print("particles before rotate:" + str(particles))
        rotate(angle, interface)
        #print("particles after rotate:" + str(particles))
        robotMeanA = sum(p[2] for p in particles) / numberOfParticles
        move(math.sqrt(math.pow(xDist, 2) + math.pow(yDist, 2)), interface)
        # print "drawParticles:" + str(particles)
        line = (robotMeanX, robotMeanY, wayPointX, wayPointY) # (x0, y0, x1, y1)
        # print "drawLine:" + str(line)
        # update mean x, mean y and mean angle
        robotMeanX = sum(p[0] for p in particles) / numberOfParticles
        robotMeanY = sum(p[1] for p in particles) / numberOfParticles
        robotMeanA = sum(p[2] for p in particles) / numberOfParticles
        print 'mean x:' + str(robotMeanX) + 'mean y:' + str(robotMeanY) + 'mean angle:' + str(robotMeanA/math.pi*180)


def readSonarSensor(interface):

    readings = []

    for i in range(0,5):
        readings.append(interface.getSensorValue(port)[0])
    return np.median(readings)

def navigateToWayPoint(wayPointX, wayPointY, interface):
    global robotMeanX, robotMeanY, robotMeanA, particles
    print ' BEFORE NAVIGATE mean x:' + str(robotMeanX) + 'mean y:' + str(robotMeanY) + 'mean angle:' + str(robotMeanA/math.pi*180)
    xDist = wayPointX - robotMeanX #- wayPointX
    yDist = wayPointY - robotMeanY #- wayPointY
    angle = math.atan2(yDist, xDist) - robotMeanA
    rotate(angle, interface)
    robotMeanA = sum(p[2] for p in particles) / numberOfParticles
    move(math.sqrt(xDist ** 2 + yDist ** 2), interface)
    line = (50 + robotMeanX * 3, 50 + robotMeanY * 3, 50 + wayPointX * 3, 50 + wayPointY * 3) # (x0, y0, x1, y1)
    # print "drawLine:" + str(line)
    # update mean x, mean y and mean angle
    robotMeanX = sum(p[0] for p in particles) / numberOfParticles
    robotMeanY = sum(p[1] for p in particles) / numberOfParticles
    robotMeanA = sum(p[2] for p in particles) / numberOfParticles
    print 'mean x:' + str(robotMeanX) + 'mean y:' + str(robotMeanY) + 'mean angle:' + str(robotMeanA/math.pi*180)




# return new angle after rotation and straight movement (angles in radian)
eSigma = 1.7
fSigma = 0.0003
gSigma = 0.0007

def getNewThetaRotate1(point, angle):
    return point[2] + angle + random.gauss(0, gSigma)

def getNewThetaStraight1(point):
    return point[2] + random.gauss(0, fSigma)

# return new x, y coordinate after moving
def getNewX1(point, distance):
    global robotMeanA
    return point[0] + (distance + random.gauss(0, eSigma)) * math.cos(robotMeanA)

def getNewY1(point, distance):
    global robotMeanA
    return point[1] + (distance + random.gauss(0, eSigma)) * math.sin(robotMeanA)

# rotate robot and update particles
def rotate(angle, interface):
    global particles
    if angle > math.pi:
        angle = angle - 2 * math.pi
    if angle < -math.pi:
        angle = angle + 2 * math.pi
    #print("angle:"+str(angle/math.pi*180))
    rot_angle = angle*rotation_angle
    interface.increaseMotorAngleReferences(motors, [rot_angle, -rot_angle])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        # if motorAngles :
        #    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    particles = [(particles[i][0], particles[i][1], getNewThetaRotate1(particles[i], angle)) for i in range(numberOfParticles)]
    # print "drawParticles:" + str([(50 + particle[0] * 3 ,50 + particle[1] * 3, 50 + particle[2] * 3) for particle in particles])
    time.sleep(0.1)
    particle_weights = [calculate_likelihood(particles[i][0], particles[i][1], particles[i][2], readSonarSensor(interface)) for i in range(numberOfParticles)]
    normaliseResample(particle_weights)
    # print "drawParticles:" + str([(50 + particle[0] * 3 ,50 + particle[1] * 3, 50 + particle[2] * 3) for particle in particles])
    time.sleep(0.1)


# move robot and update particles
def move(distance, interface):
    global particles
    move_angle = distance * drive_angle
    interface.increaseMotorAngleReferences(motors, [move_angle, move_angle])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        # if motorAngles :
        #    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    particles = [(getNewX1(particles[i], distance), getNewY1(particles[i], distance), getNewThetaStraight1(particles[i])) for i in range(numberOfParticles)]
    # print "drawParticles:" + str([(50 + particle[0] * 3 ,50 + particle[1] * 3, 50 + particle[2] * 3) for particle in particles])
    time.sleep(0.1)
    particle_weights = [calculate_likelihood(particles[i][0], particles[i][1], particles[i][2], readSonarSensor(interface)) for i in range(numberOfParticles)]
    normaliseResample(particle_weights)
    # print "drawParticles:" + str([(50 + particle[0] * 3 ,50 + particle[1] * 3, 50 + particle[2] * 3) for particle in particles])
    time.sleep(0.1)

def calculate_likelihood(x, y, theta, z):
    global walls
    print "x: " + str(x)
    print "y: " + str(y)
    closestWall = walls[0];

    ax = closestWall[0][0]
    ay = closestWall[0][1]
    bx = closestWall[1][0]
    by = closestWall[1][1]

    distFromClosestWall = math.fabs(((by-ay)*(ax-x) - (bx-ax)*(ay-y)) / ((by-ay)*math.cos(theta) - (bx-ax)*math.sin(theta)))
    for wall in walls:
        ax = wall[0][0]
        ay = wall[0][1]
        bx = wall[1][0]
        by = wall[1][1]

        distFromWall = math.fabs(((by-ay)*(ax-x) - (bx-ax)*(ay-y)) / ((by-ay)*math.cos(theta) - (bx-ax)*math.sin(theta)))
        interPtX = x + math.cos(theta) * distFromWall
        interPtY = y + math.sin(theta) * distFromWall
        beta = math.acos((math.cos(theta) * (ay - by) + math.sin(theta) * (bx - ax))/ math.sqrt((ay - by) ** 2 + (bx - ax)**2))

        if math.fabs(beta) < 0.610865:
            if interPtX < max(ax, bx) and interPtX > min(ax, bx) and interPtY < max(ay, by) and interPtY > min(ay, by):
                if distFromWall < distFromClosestWall:
                    distFromClosestWall = distFromWall
                    closestWall = wall
    # print "distance from the closest wall: " + str(distFromClosestWall)
    # print "z: " + str(z)
    # print "return value: " + str(math.exp(-math.pow((z - distFromClosestWall), 2)/(2 * 1.5)))
    return math.exp(-math.pow((z - distFromClosestWall), 2)/(2 * 1.5))

def normaliseResample(particle_weights):
    global particles
    # Normalisation
    # Add up all the weights in the unnormalised set, and then divide the weight of each by this total
    sum_set = sum(particle_weights)
    if sum_set > 0 :
        normalised_weights = [i / sum_set for i in particle_weights]

        cum_weight_array = []
        for i, w in enumerate(normalised_weights):
            if i == 0:
                cum_weight_array.append(normalised_weights[i])
            else:
                cum_weight_array.append(normalised_weights[i] + cum_weight_array[i-1])

        new_particle_set = []
        for i in range(len(particles)):
            new_particle_set.append(particles[choose_particle(cum_weight_array)])

        particles = new_particle_set[:]

def choose_particle(cum_weight_array):
    # Get random number between 0 and 1
    rand_num = random.uniform(0, 1)
    for i, val in enumerate(cum_weight_array):
        if rand_num < val :
            return i

#starting point: (84, 30)
# navigateToWayPoint(104, 30, interface)
# navigateToWayPoint(124, 30, interface)
# navigateToWayPoint(144, 30, interface)
# navigateToWayPoint(164, 30, interface)
# navigateToWayPoint(180, 30, interface)
# navigateToWayPoint(180, 50, interface)
# navigateToWayPoint(180, 54, interface)
# navigateToWayPoint(160, 54, interface)
# navigateToWayPoint(140, 54, interface)
# navigateToWayPoint(138, 54, interface)
# navigateToWayPoint(138, 74, interface)
# navigateToWayPoint(138, 94, interface)
# navigateToWayPoint(138, 114, interface)
# navigateToWayPoint(138, 134, interface)
# navigateToWayPoint(138, 154, interface)
# navigateToWayPoint(138, 168, interface)
# navigateToWayPoint(118, 168, interface)
# navigateToWayPoint(114, 168, interface)
# navigateToWayPoint(114, 148, interface)
# navigateToWayPoint(114, 128, interface)
# navigateToWayPoint(114, 108, interface)
# navigateToWayPoint(114, 88, interface)
# navigateToWayPoint(114, 84, interface)
# navigateToWayPoint(94, 84, interface)
# navigateToWayPoint(84, 84, interface)
# navigateToWayPoint(84, 64, interface)
# navigateToWayPoint(84, 44, interface)
# navigateToWayPoint(84, 30, interface)
#navigateToWayPointInput(interface)
# interface.terminate()
