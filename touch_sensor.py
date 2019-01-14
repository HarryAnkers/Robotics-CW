import brickpi
import time

touch_port_right = 1
touch_port_left = 3

interface=brickpi.Interface()
interface.initialize()

interface.sensorEnable(touch_port_right, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(touch_port_left, brickpi.SensorType.SENSOR_TOUCH)

motors = [2,1]

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
interface.setMotorAngleControllerParameters(motors[1],rightParam)

#logfile = raw_input("Give a logfile name: ")
#interface.startLogging("./logfiles/{}.txt".format(logfile))

while True:
  angle = float(input("Enter a angle to rotate (in radians): "))

  interface.increaseMotorAngleReferences(motors,[angle,angle])

  left_touched, right_touched = 0, 0
  while not interface.motorAngleReferencesReached(motors) :
    left_result = interface.getSensorValue(touch_port_left)
    for res in left_result:
        print("--------------"+str(res))
    right_result = interface.getSensorValue(touch_port_right)
    if left_result:
      print("INSIDE LEFT RESULT")
      left_touched = left_result[0]
    if right_result:
      print("INSIDE RIGHT RESULT")
      right_touched = right_result[0]

    print("LEFT TOUCHED IS: " + str(left_touched))
    print("RIGHT TOUCHED IS: " + str(right_touched))

    if left_touched and not right_touched:
        # obstacle on the left, turn right
      print("left_touched and not right_touched")
      interface.increaseMotorAngleReferences(motors,[-20,-20])
      interface.increaseMotorAngleReferences(motors,[-5,5])
    elif not left_touched and right_touched:
        # obstacle on the right, turn left
      print("not left_touched and right_touched")
      interface.increaseMotorAngleReferences(motors,[-20,-20])
      interface.increaseMotorAngleReferences(motors,[5,-5])
    elif left_touched and right_touched:
        # obstacle on the way, ???
      print("left_touched and right_touched")
      interface.increaseMotorAngleReferences(motors,[-20,-20])
    else:
      print("ELSE")
      # interface.increaseMotorAngleReferences(motors,[20,20])


    motorAngles = interface.getMotorAngles(motors)
    if motorAngles:
      print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    time.sleep(0.1)

print "Destination reached!"

interface.stopLogging()
interface.terminate()
