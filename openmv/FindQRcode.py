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

# 变量分配
Frame_Cnt = 0
fCnt_tmp = [0, 0]
count = 0
rangefinder = 0

# 局部函数定义区
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)
ir_led = LED(4)


def Find_QRcode(img):
    X = -1
    Y = -1
    Type = 0xff#未识别到返回值类型
    img.lens_corr(1.8)#图像校正
    for code in img.find_qrcodes():#查找 roi[默认整幅图像] 内的所有二维码，并返回一个 image.qrcode 对象的列表、图像上二维码需比较平展。
            img.draw_rectangle(code.rect(), color = (255, 0, 0)) # 在二维码上绘制一个矩形--->二维码矩形元组(x, y, w, h)
            X = int(code.x() + 0.5 * code.w())
            Y = int(code.y() + 0.5 * code.h())
            Type = 200#识别到返回200
    return Type, X, Y

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

# 设置像素尺寸为320*240
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
    # 记录FPS起点
    clock.tick()

    # 拍摄一张照片
    img = sensor.snapshot()

    # 寻找二维码
    (Type, X, Y) = Find_QRcode(img)

    # 打印寻找到的结果
    print(Type, X, Y)

    # 组帧，发送数据
    uart.write(UART_Send(Type, X, Y))

    # 如果寻找到了标记，则亮绿灯
    # 没有寻找到标记，则亮红灯
    if Type == 200:
        green_led.on()
        red_led.off()
    else:
        green_led.off()
        red_led.on()
        pass

    # 打印FPS信息
    print(clock.fps())
