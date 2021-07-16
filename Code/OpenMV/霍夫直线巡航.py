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

THRESHOLD = (0 ,30) # Grayscale threshold for dark things...
BINARY_VISIBLE = True
former_th = 0


Frame_Cnt = 0
fCnt_tmp = [0, 0]
count = 0
rangefinder = 0

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


def UART_Send(FormType, Loaction0, range_finder=0):
    global Frame_Cnt
    global fCnt_tmp
    # 帧头填充
    Frame_Head = [170, 170]
    # 帧尾填充
    Frame_End = [85, 85]
    # 写入看到的形状
    fFormType_tmp = FormType
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
    fFormType = bytes(int(fFormType_tmp))
    fLoaction0 = bytes(Loaction0)

    fEnd = bytes(Frame_End)
    FrameBuffe = fHead + fCnt + fFormType + fLoaction0 + fEnd
    return FrameBuffe

def KEY(lines):
    for line1 in lines:
        for line2 in lines:
            if (line1.theta()-line2.theta())==90:
                return 1
    return 0
def sub_angle(angle1,angle2):
    if abs(angle1-angle2)>90:
        return 180-abs(angle1-angle2)
    else :
        return abs(angle1-angle2)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.
uart = UART(3, 115200)

key = 0
j = 0
while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()

    # Returns a line object similar to line objects returned by find_lines() and
    # find_line_segments(). You have x1(), y1(), x2(), y2(), length(),
    # theta() (rotation in degrees), rho(), and magnitude().
    #
    # magnitude() represents how well the linear regression worked. It goes from
    # (0, INF] where 0 is returned for a circle. The more linear the
    # scene is the higher the magnitude.

    # 函数返回回归后的线段对象line，有x1(), y1(), x2(), y2(), length(), theta(), rho(), magnitude()参数。
    # x1 y1 x2 y2分别代表线段的两个顶点坐标，length是线段长度，theta是线段的角度。
    # magnitude表示线性回归的效果，它是（0，+∞）范围内的一个数字，其中0代表一个圆。如果场景线性回归的越好，这个值越大。
    i=0
    #lines = img.find_line_segments()
    #lines = img.get_regression([(200,255)],robust=True)
    lines = img.find_lines(x_stride=5)


#debug
    if (lines):
        for line1 in lines:
            img.draw_line(line1.line(), color = (127,0,0) , thichness = 1)
#debug



    if (lines):
        if KEY(lines)==1:
            key+=1
        th=lines[0].theta()
        magn=lines[0].magnitude()
        for line in lines:
            if line.magnitude()>=magn:
                th=line.theta()

            #debug
            print("FPS %f, mag = %s, theta = %s, length = %s" % (clock.fps(), str(line.magnitude()), str(line.theta()), str(line.length())))
            print(i,'\n')
            i+=1
            #debug

        if key>20:
            j=1
        print("th:",th)
'''
        if -5<=sub_angle(th,former_th)<=5 or sub_angle(th,former_th)<=-65 or sub_angle(th,former_th)>=65:
            print("ONE")
            uart.write(UART_Send(former_th, j))
            former_th = former_th
        elif -20<=sub_angle(th,former_th)<=20:
            print("TWO")
            uart.write(UART_Send((th+former_th)/2, j))
            former_th = (th+former_th)/2
        else:
            print("THREE")
            uart.write(UART_Send(th, j))
            former_th = th
        print("result:",former_th)
'''
