from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 8

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
    motors[0].setVelocity(val)
    motors[2].setVelocity(val)

def setRightWheel(val):
    motors[1].setVelocity(val)
    motors[3].setVelocity(val)


def forward(val):
    setLeftWheel(val)
    setRightWheel(val)

def turn_left(val):
    setLeftWheel(val)
    setRightWheel(-val)

def turn_right(val):
    setLeftWheel(-val)
    setRightWheel(val)


max_vel = 20

while robot.step(timestep) != -1:
    traces = []
    left_vel = 0.0
    right_vel = 0.0
    for name, sensor in zip(ds_name, sensors):
        traces.append(getTraceVal(sensor))

    print(traces)
    case1_1 = traces[0]==1 and traces[1]==0 and traces[2]==0 and traces[3]==1 and traces[4]==1
    case1_2 = traces[0]==1 and traces[1]==1 and traces[2]==0 and traces[3]==0 and traces[4]==1
    case1_3 = traces[0]==1 and traces[1]==0 and traces[2]==0 and traces[3]==0 and traces[4]==1

    case2_1 = traces[0]==0 and traces[1]==0 and traces[2]==1 and traces[3]==1 and traces[4]==1
    case2_2 = traces[0]==0 and traces[1]==1 and traces[2]==1 and traces[3]==1 and traces[4]==1
    
    case3_1 = traces[0]==1 and traces[1]==1 and traces[2]==1 and traces[3]==0 and traces[4]==0
    case3_2 = traces[0]==1 and traces[1]==1 and traces[2]==1 and traces[3]==1 and traces[4]==0
    
    case4 = traces[0]==0 and traces[1]==0 and traces[2]==0 and traces[3]==0 and traces[4]==0
    case5 = traces[0]==1 and traces[1]==1 and traces[2]==1 and traces[3]==1 and traces[4]==1

    if case1_1 or case1_2 or case1_3:
        forward(max_vel*0.5)
    elif case2_1 or case2_2:
        turn_right(max_vel*0.5)
    elif case3_1 or case3_2:
        turn_left(max_vel*0.5)
    else:
        forward(max_vel*0.5)




# Enter here exit cleanup code.
