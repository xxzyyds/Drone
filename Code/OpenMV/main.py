#导入外部包
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

# 全局变量分配
Frame_Cnt = 0
fCnt_tmp = [0, 0]
count = 0
rangefinder = 0
#LAB
#lmin，lmax ,
black = ((0, 20, -13, 42, -128, 127))
#yellow = (51, 76, -17, 10, 47, 77)//黄色纸张
yellow = (17, 69, -13, 6, -128, 127)#黄色胶带（下午）
num_black = 0
num_yellow = 0
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

# 串口3 P4 P5 = TX RX
# 串口1 P1 P2 = TX RX

# 根据得到的坐标信息，通过串口发送数据


def UART_Send(FormType, Loaction0, Location1, range_finder=0):
    global Frame_Cnt
    global fCnt_tmp
    # 帧头填充
    Frame_Head = [170, 170]
    # 帧尾填充
    Frame_End = [85, 85]
    # 写入看到的形状
    fFormType_tmp = [FormType]
    # FrameCnt自动累加，识别不同的帧
    Frame_Cnt += 1

    if Frame_Cnt > 65534:
        FrameCnt = 0

    fHead = bytes(Frame_Head)

    fCnt_tmp[0] = range_finder & 0xFF
    fCnt_tmp[1] = range_finder >> 8
    # if(range_finder != 0):
    #     fCnt_tmp[1] = range_finder
    fCnt = bytes(fCnt_tmp)
    # 拆分长帧
    fFormType = bytes(fFormType_tmp)
    fLoaction0 = bytes(ExceptionVar(Loaction0))
    fLoaction1 = bytes(ExceptionVar(Location1))
    fEnd = bytes(Frame_End)
    FrameBuffe = fHead + fCnt + fFormType + fLoaction0 + fLoaction1 + fEnd
    return FrameBuffe

# main函数区
# 复位传感器
sensor.reset()

# 设置图片格式为RGB--返回元组
sensor.set_pixformat(sensor.RGB565)

# 设置像素尺寸160*120
sensor.set_framesize(sensor.QQVGA)

# 跳过图像不稳定的前2s的图像
sensor.skip_frames(time=2000)

# 关闭自动增益和自动白平衡
sensor.set_auto_gain(False)  #  必须关闭此功能，以防止图像冲洗
sensor.set_auto_whitebal(False)  # must be turned off for color tracking

# 初始化时钟和串口
clock = time.clock()
uart = UART(3, 115200)
while(True):
    action1 = 0
    action2 = 0
    key = 0
    Send = False

    # 记录FPS起点
    clock.tick()

    # 拍摄一张照片
    img = sensor.snapshot()
    # 寻找矩形
    rects = img.find_rects(threshold=20000)
    if rects:
        for rect in rects:
            if rect:
                pass
            else:
                rect = (0,0,160,120)#如果改像素尺寸这里也要改
            # 寻找色块(x_stride和area_threshold的值可能还要进行修改）
            #img.draw_rectangle(rect.rect(),color=(255,0,0))
            black_blobs = img.find_blobs([black],roi = rect.rect(), x_stride=5, y_stride=3, invert=False, area_threshold=200, pixels_threshold=200, merge=False)
            yellow_blobs = img.find_blobs([yellow],roi = rect.rect(), x_stride=5, y_stride=3, invert=False, area_threshold=200, pixels_threshold=200, merge=False)
            for blob in black_blobs:
                if (blob.w()/blob.h()) <= 5:     #后续确定好飞机高度以及与杆的距离之后还要修改长宽比
                    if 80<= blob.cx() <= 90:     #设置一个值还是一个范围有待商榷
                        num_black += 1
                        img.draw_rectangle(blob.rect(),color=(255,0,0))
                        break
            for blob in yellow_blobs:
                if (blob.w()/blob.h()) <= 5:
                    if 80<=blob.cx() <= 90:   #设置一个值还是一个范围有待商榷
                        num_yellow += 1
                        img.draw_rectangle(blob.rect(),color=(255,0,0))
                        break

    if num_black == 1 or num_yellow == 1 or num_yellow == 2:
        key = 200
    elif num_black == 2 or num_black == 3:
        key = 200
        atcion1 = 100
    elif num_black == 4:
        key = 200
        atcion1 = 100
        action2 = 100

    # 组帧，发送数据
    uart.write(UART_Send(key, action1, action2))
    # 打印FPS信息
    print(key , action1 , action2 , num_black , num_yellow)
