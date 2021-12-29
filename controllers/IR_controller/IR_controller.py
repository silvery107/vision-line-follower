from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64

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


def getError(pos, ID_data):
    online = 0.0
    PosX = 0.0
    for i in range(len(ID_data)):
        if ID_data[i]:
            PosX += i
            online += 1

    # If outline sensor, retur the latest position
    if online == 0:
        return pos

    return PosX / online - 2


MAX_VEL = 40
KP = 10
KD = 0.01  #0.12
KI = 0.001
pos = 0.0
err = 0.0
err_D = 0.0
err_I = 0.0
left_vel = 0.0
right_vel = 0.0
while robot.step(timestep) != -1:
    ID_data = [getIRValue(sensor) for sensor in sensors]
    # print(traces)

    err = getError(pos, ID_data)
    err_I += err # zero this when outline
    PD_feedback = KP * err + KD * (err - err_D) + KI * err_I
    err_D = err

    left_vel = 0.5 * MAX_VEL - PD_feedback
    right_vel = 0.5 * MAX_VEL + PD_feedback
    # print(left_vel, right_vel)

    setLeftWheel(left_vel)
    setRightWheel(right_vel)

# Enter here exit cleanup code.
