from picamera import PiCamera
from time import sleep, time
import numpy as np
import threading
import serial
import cv2

# setup camera
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 24
camera.rotation = 0
camera.brightness = 50
sleep(1)

# setup serial GPIO 14 TX GPIO 15 RX
ser = serial.Serial("/dev/ttyAMA0", 19200, timeout = 0.005)
if ser.isOpen() == False:
    ser.open()

# parameters
MAX_VEL = 100
KP = 20
KD = 1
LINE_START = 150
LINE_COUNT = 50

# main loop
error = 0.0
last_error = 0.0
img = np.empty((240, 320, 3), dtype=np.uint8)
t = time()
start_flag = True
count = 0
line = ""
try:
    while True:
#        if count % 100 == 0:
#            line = ser.read(5).decode("utf-8")
#            print(line)
        line = ser.read(5).decode("utf-8")
        print(line)
        if count > 1e6:
            count = 0
            
        if "END" in line:
            #print("find END")
            error = 0.0
            last_error = 0.0
            start_flag = False
            
        elif "STA" in line:
            #print("find STA")
            error = 0.0
            last_error = 0.0
            start_flag = True
            
        if start_flag:
            camera.capture(img, format='rgb', use_video_port=True)
            img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            #img_gray = cv2.erode(img_gray,
            #                     cv2.getStructuringElement(cv2.MORPH_ERODE, (4, 4)),
            #                     iterations=1)
            _, img_thr = cv2.threshold(img_gray, 0, 255, cv2.THRESH_OTSU)
        
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
                #line_center = 160
                continue
        
            error = (line_center - 160) / 160
            PD_feedback = KP * error + KD * (error - last_error)
            ser_data = 	"{:.5f}".format(PD_feedback) + "A"
            ser.write(ser_data.encode('utf-8'))
            
            #print(ser_data.encode('utf-8'))
        
        count += 1
        sleep(0.005) # 100 Hz
except KeyboardInterrupt:
    ser.close()
    camera.close()
    
#cv2.imwrite("./test_thr.jpeg", img_thr)
