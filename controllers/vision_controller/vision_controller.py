from controller import Robot
import numpy as np
import cv2
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64
motor_name = ["wheel1", "wheel2", "wheel3", "wheel4"]
ds_name = ["ds_1", "ds_2", "ds_3", "ds_4", "ds_5"]
motors = []
sensors = []
camera = robot.getDevice("camera")  # (240, 320, 3)
camera.enable(timestep)

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
    if temp > 500:
        return 0
    else:
        return 1


def setLeftWheel(val):
    if val > MAX_VEL:
        val = MAX_VEL
    motors[0].setVelocity(val)
    motors[2].setVelocity(val)


def setRightWheel(val):
    if val > MAX_VEL:
        val = MAX_VEL
    motors[1].setVelocity(val)
    motors[3].setVelocity(val)


win_name = "line"
MAX_VEL = 50
KP = 20
KD = 1
LINE_START = 100
LINE_COUNT = 100

error = 0.0
last_error = 0.0
while robot.step(timestep) != -1:
    traces = []
    left_vel = 0.5 * MAX_VEL
    right_vel = 0.5 * MAX_VEL

    for name, sensor in zip(ds_name, sensors):
        traces.append(getTraceVal(sensor))
    # print(traces)

    img = np.array(camera.getImageArray()).astype(np.uint8)  # (240, 320, 3)
    img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img_gray = cv2.rotate(cv2.flip(img_gray, 0), cv2.ROTATE_90_CLOCKWISE)
    img_gray = cv2.erode(img_gray,
                         cv2.getStructuringElement(cv2.MORPH_ERODE, (5, 5)),
                         iterations=2)
    _, img_thr = cv2.threshold(img_gray, 0, 255, cv2.THRESH_OTSU)
    # print(img_thr.shape)
    # cv2.imshow(win_name, img_thr)
    # cv2.waitKey(27)
    centers = []

    for i in range(LINE_COUNT):
        line = img_thr[LINE_START + i, :]
        black_count = np.sum(line == 0)
        black_index = np.where(line == 0)[0]  # uppack tuple (320,)
        if black_count == 0:
            black_count = 1

        if black_index.size == 0:
            continue

        center = (black_index[0] + black_index[black_count - 1]) / 2
        centers.append(center)

    if len(centers) != 0:
        line_center = np.sum(centers) / len(centers)
    else:
        continue
        # line_center = 160

    error = (line_center - 160) / 160
    PD_feedback = KP * error + KD * (error - last_error)
    last_error = error
    print(PD_feedback)

    left_vel += PD_feedback
    right_vel += -PD_feedback
    # print(left_vel, right_vel)
    setLeftWheel(left_vel)
    setRightWheel(right_vel)

# Enter here exit cleanup code.
