# 追踪颜色（黄色） By: 中科浩电 - 周一 3月 22 2021

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

# 变量分配区
# 黄色追踪
yellow = [43, 63, -17, 15, 21, 67]
Frame_Cnt = 0
fCnt_tmp = [0, 0]
count = 0
rangefinder = 0
area_tmp = [0, 0]
area = 0
# 局部函数定义区

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)
ir_led = LED(4)

min_degree = 0
max_degree = 20

enable_lens_corr = False # 不矫正畸变
def get_rangefinder(uart_line):
    try:
        if (len(uart_line) < 8):
            return 0xFF
        if(uart_line[0] != 0x59):
            return 0xFF
        if(uart_line[1] != 0x59):
            return 0xFF
        if(uart_line[2] != None and uart_line[3] != None):
            return uart_line[3] * 256 | uart_line[2]
    except:
        return 20
        pass


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

# 串口3 P4 P5 = TX RX
# 串口1 P1 P2 = TX RX


def UART_Send(FormType, Loaction0, Location1, range_finder=0, frame_type=0, area=0, prame2=0, prame3=0, prame4=0):
    global Frame_Cnt
    global fCnt_tmp
    Frame_Head = [170, 170]
    Frame_End = [85, 85]
    fFormType_tmp = [FormType]
    Frame_Cnt += 1
    prame = bytes(0)

    if Frame_Cnt > 65534:
        FrameCnt = 0

    fHead = bytes(Frame_Head)

    # 计算距离值
    fCnt_tmp[0] = range_finder & 0xFF
    fCnt_tmp[1] = range_finder >> 8
    fCnt = bytes(fCnt_tmp)

    # 计算区域值
    area_tmp[0] = area & 0xFF
    area_tmp[1] = area >> 8
    area = bytes(area_tmp)

    fFormType = bytes(fFormType_tmp)
    fLoaction0 = bytes(ExceptionVar(Loaction0))
    fLoaction1 = bytes(ExceptionVar(Location1))
    fEnd = bytes(Frame_End)

    if frame_type == 0:
        FrameBuffe = fHead + fCnt + fFormType + fLoaction0 + fLoaction1 + fEnd
    elif frame_type == 1:
        FrameBuffe = fHead + fCnt + fFormType + fLoaction0 + \
            fLoaction1 + area + prame + prame + prame + fEnd
        pass
    return FrameBuffe


# main函数区
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()
uart = UART(3, 115200)
uart_Rangefinder = UART(1, 115200)
cnt = 0
cnt_temp = 0

tag_families = 0
tag_families |= image.TAG25H7 # comment out to disable this family

while(True):
    clock.tick()
    img = sensor.snapshot()
    #if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...

    # `threshold` controls how many lines in the image are found. Only lines with
    # edge difference magnitude sums greater than `threshold` are detected...

    # More about `threshold` - each pixel in the image contributes a magnitude value
    # to a line. The sum of all contributions is the magintude for that line. Then
    # when lines are merged their magnitudes are added togheter. Note that `threshold`
    # filters out lines with low magnitudes before merging. To see the magnitude of
    # un-merged lines set `theta_margin` and `rho_margin` to 0...

    # `theta_margin` and `rho_margin` control merging similar lines. If two lines
    # theta and rho value differences are less than the margins then they are merged.

    #for l in img.find_lines(threshold = 1000, theta_margin = 50, rho_margin = 25):
    #    if (min_degree <= l.theta()) and (l.theta() <= max_degree):
    #        img.draw_line(l.line(), color = (255, 0, 0))
    #        Type = 205
            #lines = img.find_lines(threshold = 1000, theta_margin = 50, rho_margin = 50)
    #        if(l.x1() < 160 and Type == 205):

    #            uart.write(UART_Send(Type,l.x1(),l.y1()))
                #print(l.x1())
            # print(l)




    blobs = img.find_blobs([yellow], pixels_threshold=100,area_threshold=100, merge=True, margin=10)
    if blobs:
        green_led.on()
        red_led.off()
        p = []
        p.append(blobs[0])
        for blob in p:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            P0 = blob.cx()
            P1 = blob.cy()
            area = blob.area()
            Type = 201

    else:
        green_led.off()
        red_led.on()
        Type = 0xFF
        P0 = -1
        P1 = -1
    if Type == 201 and P0 <160:
        uart.write(UART_Send(Type, P0, P1, rangefinder, area = area, frame_type=1))
    if (count % 2 == 0):
        if (uart_Rangefinder.any()):
            p = uart_Rangefinder.readline()
            # print(bytes(p))
            rangefinder = get_rangefinder(p)
            cnt+=1
            print(rangefinder)
        if cnt != cnt_temp:
            blue_led.on()
            pass
        else:
            blue_led.off()

        cnt_temp = cnt
    count += 1

    # print(clock.fps())
