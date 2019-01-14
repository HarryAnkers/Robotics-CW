import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]
speed = 6.0

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
leftParam.pidParameters.k_p = 100.0
leftParam.pidParameters.k_i = 0.0
leftParam.pidParameters.k_d = 0.0

rightParam.maxRotationAcceleration = 6.0
rightParam.maxRotationSpeed = 12.0
rightParam.feedForwardGain = 255/20.0
rightParam.minPWM = 18.0
rightParam.pidParameters.minOutput = -255
rightParam.pidParameters.maxOutput = 255
rightParam.pidParameters.k_p = 100.0
rightParam.pidParameters.k_i = 0.0
rightParam.pidParameters.k_d = 0.0

interface.setMotorAngleControllerParameters(motors[0],leftParam)
interface.setMotorAngleControllerParameters(motors[1],rightParams)

interface.setMotorRotationSpeedReferences(motors,[speed,speed])

interface.startLogging("./logfile.txt")

print "Press Ctrl+C to exit"
while True:
	time.sleep(1)

interface.stopLogging()
interface.terminate()
