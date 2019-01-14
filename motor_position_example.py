import brickpi
import time

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

logfile = raw_input("Give a logfile name: ")
logfilepath = "./logfiles/{}.txt".format(logfile)

while True:
	angle = float(input("Enter a angle to rotate (in radians): "))

        interface.startLogging(logfilepath)
	interface.increaseMotorAngleReferences(motors,[angle,-angle])

	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)
	
	print "Destination reached!"

interface.stopLogging()

interface.terminate()

