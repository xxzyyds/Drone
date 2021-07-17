import sensor
import image
import time
import network
import usocket
import sys
import sensor
import image
import time
import network
import usocket
import sys
import math
from pyb import UART
from pyb import LED
Frame_Cnt = 0
fCnt_tmp = [0, 0]
count = 0
rangefinder = 0
black = (0, 53, -45, 61, -101, 12)
yellow = (68, 87, -40, 2, 8, 127)

last_num = 0
num = 0
i = 0

def ExceptionVar(var):
    data = []
    data.append(0)
    data.append(0)
    if var == -1:
        data[0] = 0
        data[1] = 0
    else:
        data[0] = var & 0xFF
        data[1] = var >> 8
    return data
def UART_Send(FormType, Loaction0, Location1, range_finder=0):
    global Frame_Cnt
    global fCnt_tmp
    Frame_Head = [170, 170]
    Frame_End = [85, 85]
    fFormType_tmp = [FormType]
    Frame_Cnt += 1
    if Frame_Cnt > 65534:
        FrameCnt = 0
    fHead = bytes(Frame_Head)
    fCnt_tmp[0] = range_finder & 0xFF
    fCnt_tmp[1] = range_finder >> 8
    fCnt = bytes(fCnt_tmp)
    fFormType = bytes(fFormType_tmp)
    fLoaction0 = bytes(ExceptionVar(Loaction0))
    fLoaction1 = bytes(ExceptionVar(Location1))
    fEnd = bytes(Frame_End)
    FrameBuffe = fHead + fCnt + fFormType + fLoaction0 + fLoaction1 + fEnd
    return FrameBuffe

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
uart = UART(3, 115200)

while(True):

    action1 = 0
    action2 = 0
    key = 0

    clock.tick()
    img = sensor.snapshot()
    rects = img.find_rects(threshold=20000)

    if rects:
        for rect in rects:
            if rect:
                pass
            else:
                rect = (0,0,160,120)
            black_blobs = img.find_blobs([black],roi = rect.rect(), x_stride=5, y_stride=3, invert=False, area_threshold=200, pixels_threshold=200, merge=False)
            yellow_blobs = img.find_blobs([yellow],roi = rect.rect(), x_stride=5, y_stride=3, invert=False, area_threshold=200, pixels_threshold=200, merge=False)
            for blob in black_blobs:
                if (blob.w()/blob.h()) <= 1:
                    if 80<= blob.cx() <= 90:
                        num += 1
                        img.draw_rectangle(blob.rect(),color=(255,0,0))
                        green_led.on()
                        break
            for blob in yellow_blobs:
                if (blob.w()/blob.h()) <= 5:
                    if 80 <= blob.cx() <= 90:
                        num += 1
                        img.draw_rectangle(blob.rect(),color=(255,0,0))
                        green_led.on()
                        break
    red_led.on()
    #if num - last_num  == 1:
        #key = 200
        #last_num += 1
    #elif num - last_num == 2 or num - last_num == 3:
        #key = 200
        #atcion1 = 100
        #last_num += 1
    #elif num - last_num == 4:
        #key = 200
        #atcion1 = 100
        #action2 = 100
        #last_num += 1
    uart.write(UART_Send(100, 100, 100))
    #print(key , action1 , action2)
