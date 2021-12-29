from controller import Robot
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 32

ds_name = ["ds_1", "ds_2", "ds_3", "ds_4", "ds_5"]
motor_name = ["wheel1", "wheel2", "wheel3", "wheel4"]
motors = []
sensors = []
for name in motor_name:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))  # Velocity control mode.
    motor.setVelocity(0.0)
    motors.append(robot.getDevice(name))

for name in ds_name:
    ds = robot.getDevice(name)
    ds.enable(timestep)
    sensors.append(ds)


def getIRValue(ds):
    temp = ds.getValue()
    if temp > 500:
        return 0
    else:
        return 1


def setLeftWheel(val):
    if val > MAX_VEL:
        val = MAX_VEL
    if val < -MAX_VEL:
        val = -MAX_VEL
    motors[0].setVelocity(val)
    motors[2].setVelocity(val)


def setRightWheel(val):
    if val > MAX_VEL:
        val = MAX_VEL
    if val < -MAX_VEL:
        val = -MAX_VEL
    motors[1].setVelocity(val)
    motors[3].setVelocity(val)


def findMaxConsecutiveZeros(nums):
    """
    :type nums: List[int]
    :rtype: int
    """
    if len(nums) == 0:
        return None
    index = 0
    times = 0

    for i in range(len(nums)):
        if nums[i] == 1:
            times = max(times, i - index)
            index = i + 1

    times = max(times, len(nums) - index)
    return times


def getError(pos, ID_data):

    online = 0.0
    PosX = 0.0

    # overcome cross line
    if findMaxConsecutiveZeros(ID_data) >= 3:
        return 0

    for i in range(len(ID_data)):
        if ID_data[i]:
            PosX += i
            online += 1

    # If outline sensor, retur the latest position
    if online == 0:
        return pos

    # 2 means the mid idx of 5 IR
    return PosX / online - 2


MAX_VEL = 40
KP = 15
KD = 0.1
KI = 0.001
pos = 0.0
err = 0.0
err_D = 0.0
err_I = 0.0
left_vel = 0.0
right_vel = 0.0
while robot.step(timestep) != -1:
    IR_data = [getIRValue(sensor) for sensor in sensors]
    # print(IR_data)

    err = getError(pos, IR_data)
    err_I += err  # zero this when outline
    if robot.getTime() % 10 == 0:
        err_I = 0
    PD_feedback = KP * err + KD * (err - err_D) + KI * err_I
    err_D = err

    left_vel = 0.5 * MAX_VEL - PD_feedback
    right_vel = 0.5 * MAX_VEL + PD_feedback
    # print(left_vel, right_vel)

    setLeftWheel(left_vel)
    setRightWheel(right_vel)

# Enter here exit cleanup code.
