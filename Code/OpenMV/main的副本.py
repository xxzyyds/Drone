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
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)
ir_led = LED(4)
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
def Find_QRcode(img):
	X = -1
	Y = -1
	FormType = 0xff
	text = 99
	img.lens_corr(1.8)
	for code in img.find_qrcodes():
		img.draw_rectangle(code.rect(), color=(150))
		code_cx = int(code.x() + 0.5 * code.w())
		code_cy = int(code.y() + 0.5 * code.h())
		img.draw_cross(code_cx, code_cy, color=(150))
		X = code_cx
		Y = code_cy
		FormType = 100
	return FormType, X, Y
def Find_Apriltags(img):
	X = -1
	Y = -1
	FormType = 0xff
	for tag in img.find_apriltags(families=image.TAG16H5):
		img.draw_rectangle(tag.rect(), color=(150))
		img.draw_cross(tag.cx(), tag.cy(), color=(150))
		X = tag.cx()
		Y = tag.cy()
		FormType = 100
	return FormType, X, Y
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
sensor.skip_frames(30)
sensor.set_auto_gain(False)
clock = time.clock()
uart = UART(3, 115200)
while(True):
	clock.tick()
	img = sensor.snapshot()
	(Type, P0, P1) = Find_QRcode(img)
	print(Type, P0, P1)
	if Type == 100:
		green_led.on()
		red_led.off()
	else:
		green_led.off()
		red_led.on()
		pass
	print(clock.fps())