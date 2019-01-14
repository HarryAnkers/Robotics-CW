import brickpi
import time
import sys
import random
import math
import numpy as np
import wayPoint2
import histogram2

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

###################### wayPoint nagivation for each point #################

def start_at_one(angle):
    # set starting point
    wayPoint2.set_starting_point(84, 30, angle)
    wayPoint2.navigateToWayPoint(180, 30, interface)
    wayPoint2.navigateToWayPoint(180, 54, interface)
    wayPoint2.navigateToWayPoint(138, 54, interface)
    wayPoint2.navigateToWayPoint(138, 168, interface)
    wayPoint2.navigateToWayPoint(84, 30, interface)
def start_at_two(angle):
    # set starting Point
    wayPoint2.set_starting_point(180, 30, angle)
    wayPoint2.navigateToWayPoint(180, 54, interface)
    wayPoint2.navigateToWayPoint(138, 54, interface)
    wayPoint2.navigateToWayPoint(138, 168, interface)
    wayPoint2.navigateToWayPoint(84, 30, interface)
    wayPoint2.navigateToWayPoint(180, 30, interface)
def start_at_three(angle):
    wayPoint2.set_starting_point(180, 54, angle)
    wayPoint2.navigateToWayPoint(138, 54, interface)
    wayPoint2.navigateToWayPoint(138, 168, interface)
    wayPoint2.navigateToWayPoint(84, 30, interface)
    wayPoint2.navigateToWayPoint(180, 30, interface)
    wayPoint2.navigateToWayPoint(180, 54, interface)
def start_at_four(angle):
    wayPoint2.set_starting_point(138, 54, angle)
    wayPoint2.navigateToWayPoint(138, 168, interface)
    wayPoint2.navigateToWayPoint(84, 30, interface)
    wayPoint2.navigateToWayPoint(180, 30, interface)
    wayPoint2.navigateToWayPoint(180, 54, interface)
    wayPoint2.navigateToWayPoint(138, 54, interface)
def start_at_five(angle):
    wayPoint2.set_starting_point(138, 168, angle)
    wayPoint2.navigateToWayPoint(84, 30, interface)
    wayPoint2.navigateToWayPoint(180, 30, interface)
    wayPoint2.navigateToWayPoint(180, 54, interface)
    wayPoint2.navigateToWayPoint(138, 54, interface)
    wayPoint2.navigateToWayPoint(138, 168, interface)

################## final assessment ####################
# Point information
p1 = (84, 30)
p2 = (180, 30)
p3 = (180, 54)
p4 = (138, 54)
p5 = (138, 168)

# Get initial start location

start_array = histogram2.location_obs(interface)
start_location = histogram2.get_estimate_location(histogram2.return_depth_hist(start_array.sig))
start_orientation = histogram2.recognize_orientation(start_array, histogram2.signatures.read(start_location))

# Move
if start_location == 0:
    start_at_one(start_orientation)
elif start_location == 1:
    start_at_two(start_orientation)
elif start_location == 2:
    start_at_three(start_orientation)
elif start_location == 3:
    start_at_four(start_orientation)
else:
    start_at_five(start_orientation)

interface.terminate()
