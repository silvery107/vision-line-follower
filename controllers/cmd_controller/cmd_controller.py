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

def getTraceVal(ds):
    temp = ds.getValue()
    if temp>500:
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

def forward(val):
    setLeftWheel(val)
    setRightWheel(val)

def turnLeft(val):
    setLeftWheel(-val)
    setRightWheel(val)

def turnRight(val):
    setLeftWheel(val)
    setRightWheel(-val)

def turnBack(val):
    setLeftWheel(-val)
    setRightWheel(-val)


MAX_VEL = 40

while robot.step(timestep) != -1:
    traces = []
    left_vel = 0.0
    right_vel = 0.0
    for name, sensor in zip(ds_name, sensors):
        traces.append(getTraceVal(sensor))

    # print(traces)

	# Forward:
    case1_1 = traces[0]==1 and traces[1]==0 and traces[2]==0 and traces[3]==1 and traces[4]==1
    case1_2 = traces[0]==1 and traces[1]==1 and traces[2]==0 and traces[3]==0 and traces[4]==1
    case1_3 = traces[0]==1 and traces[1]==0 and traces[2]==0 and traces[3]==0 and traces[4]==1
    # Turn Right:
    case2_1 = traces[0]==0 and traces[1]==0 and traces[2]==1 and traces[3]==1 and traces[4]==1
    case2_2 = traces[0]==0 and traces[1]==1 and traces[2]==1 and traces[3]==1 and traces[4]==1
    case2_3 = traces[0]==1 and traces[1]==0 and traces[2]==1 and traces[3]==1 and traces[4]==1 # out2 black - small correction 
    case2_4 = traces[0]==0 and traces[1]==0 and traces[2]==0 and traces[3]==1 and traces[4]==1
    case2_5 = traces[0]==0 and traces[1]==0 and traces[2]==0 and traces[3]==0 and traces[4]==1
    case2_6 = traces[0]==0 and traces[1]==1 and traces[2]==0 and traces[3]==1 and traces[4]==1
    # Turn Left:
    case3_1 = traces[0]==1 and traces[1]==1 and traces[2]==1 and traces[3]==0 and traces[4]==0
    case3_2 = traces[0]==1 and traces[1]==1 and traces[2]==1 and traces[3]==1 and traces[4]==0
    case3_3 = traces[0]==1 and traces[1]==1 and traces[2]==1 and traces[3]==0 and traces[4]==1 # out4 black - small correction 
    case3_4 = traces[0]==1 and traces[1]==1 and traces[2]==0 and traces[3]==0 and traces[4]==0
    case3_5 = traces[0]==1 and traces[1]==0 and traces[2]==0 and traces[3]==0 and traces[4]==0
    case3_6 = traces[0]==1 and traces[1]==1 and traces[2]==0 and traces[3]==0 and traces[4]==0
    # Others
    case4 = traces[0]==0 and traces[1]==0 and traces[2]==0 and traces[3]==0 and traces[4]==0
    case5 = traces[0]==1 and traces[1]==1 and traces[2]==1 and traces[3]==1 and traces[4]==1

    if case1_1 or case1_2 or case1_3:
        forward(MAX_VEL*0.5)
    elif case2_1 or case2_2:
        turnLeft(MAX_VEL*0.5)
    elif case2_3 or case2_4 or case2_5:
        turnLeft(MAX_VEL*0.6)
    elif case3_1 or case3_2:
        turnRight(MAX_VEL*0.4)
    elif case3_3 or case3_4 or case3_5:
        turnRight(MAX_VEL*0.6)
    elif case4:
        # used for counting cross line and end point
        pass
    elif case5:
        turnBack(MAX_VEL*0.6)
    else:
        forward(MAX_VEL*0.4)




# Enter here exit cleanup code.
