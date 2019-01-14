import brickpi
import time
import sys
import random
import math
import numpy as np

interface=brickpi.Interface()
interface.initialize()

motors = [1,2,0]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
interface.motorEnable(motors[2])

leftParam = interface.MotorAngleControllerParameters()
rightParam = interface.MotorAngleControllerParameters()
sonarParam = interface.MotorAngleControllerParameters()

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

sonarParam.maxRotationAcceleration = 6.0
sonarParam.maxRotationSpeed = 12.0
sonarParam.feedForwardGain = 255/20.0
sonarParam.minPWM = 18.0
sonarParam.pidParameters.minOutput = -255
sonarParam.pidParameters.maxOutput = 255
sonarParam.pidParameters.k_p = 462.0
sonarParam.pidParameters.k_i = 800.0
sonarParam.pidParameters.K_d = 23.0

interface.setMotorAngleControllerParameters(motors[0],leftParam)
interface.setMotorAngleControllerParameters(motors[1],rightParam)
interface.setMotorAngleControllerParameters(motors[2],sonarParam)


port = 0 # port which ultrasoic sensor is plugged in to

interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)

#initX = 0
#initY = 0
#initA = 0

initX = 84
initY = 30
initA = 0
rotation_angle = 1.7

walls = [((0,0), (0,168)), ((0,168), (84, 168)), ((84, 168), (84, 126)), ((84, 126), (84, 210)), ((84, 210), (168, 210)), ((168, 210), (168, 84)), ((168, 84), (210, 84)), ((210, 84), (210, 0)), ((210, 0), (0, 0))]


######## sweep #########

robotMeanX, robotMeanY, robotMeanA = 84, 30, 0 #initial position


def sweep():
	global robotMeanX, robotMeanY, robotMeanA
	robotSonarReadings=[]

	for i in range(72):

# 0.087
		rotate(0.0515, interface)
		robotSonarReadings.append(readSonarSensor())


	for i in range(len(robotSonarReadings)):
		print str(robotSonarReadings[i])

	interface.terminate()

	return robotSonarReadings

	#for other group here use this array to compare with points using formula on lecture slides

	#take highest variance and that is the correct x and y

	#do a for loop to find what shift to apply to our readings gets highest accuracy on the estimated point

	#using new x y and angle estimate print lines at point

def readSonarSensor():
    readings = []

	# get the median of 5 readings
    for i in range(0,5):
        readings.append(interface.getSensorValue(port)[0])
    return np.median(readings)

# rotate robot and update particles
def rotate(angle,interface):
    if angle > math.pi:
        angle = angle - 2 * math.pi
    if angle < -math.pi:
        angle = angle + 2 * math.pi
    #print("angle:"+str(angle/math.pi*180))
    rot_angle = angle*rotation_angle
    interface.increaseMotorAngleReferences(motors, [0, 0, rot_angle])

    while not interface.motorAngleReferencesReached(motors):
        motorAngles = interface.getMotorAngles(motors)
        # if motorAngles :
        #    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]

#starting point: (84, 30)
#navigateToWayPointInput(interface)
