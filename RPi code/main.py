from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import serial
from pyzbar.pyzbar import decode
from time import sleep
import numpy as np
import math

resolution = [640, 480]
horizontal_center = resolution[0]//2
vertical_center = resolution[1]//2
threshold = 50

widthImg = 640
heightImg = 480



def check_x(_sos):
	#empirically obtained formula
    distance_x_axis = 19667/ _sos - 70
    distance_x_axis = str(distance_x_axis)
    print(str(_sos) + " "+ distance_x_axis)
    return distance_x_axis


def check_y(_polygon):
    if centroid(_polygon)[0] > (horizontal_center + threshold):
        return "Y-\n\r"  # Move right from camera's perspective
    elif centroid(_polygon)[0] < (horizontal_center - threshold):
        return "Y+\n\r"  # Move left from camera's perspective
    else:
        return "Y=0\n\r"


def check_z(_polygon):
    if centroid(_polygon)[1] > (vertical_center + threshold):
        return "z=-1\n\r"  # Move down from camera's perspective
    elif centroid(_polygon)[1] < (vertical_center - threshold):
        return "z=1\n\r"  # Move up from camera's perspective
    else:
        return "z=0\n\r"

def centroid(vertexes):
     _x_list = [vertex [0] for vertex in vertexes]
     _y_list = [vertex [1] for vertex in vertexes]
     _len = len(vertexes)
     _x = sum(_x_list) / _len
     _y = sum(_y_list) / _len
     return(_x, _y)


def calculateDistance(x1,y1,x2,y2):
     dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
     return dist

#commands to be sent to Arduino
X = ''
Y = ''
Z = ''

X_set = False
Z_set = False
await_x_ok = False

#serial initialization
ser = serial.Serial('/dev/serial0',9600,timeout=1)
ser.flush()

#camera initialization
camera = PiCamera()
camera.resolution = (resolution[0],resolution[1])
rawCapture = PiRGBArray(camera, size=(resolution[0],resolution[1]))

if __name__ == '__main__':
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		img = frame.array
		imgQR = img.copy()
		print(decode(img))
		qr_data = decode(img)
		if qr_data:
			polygon= qr_data[0].polygon
			cv2.polylines(imgQR,pts=np.int32([polygon]),isClosed=True,color=(0,0,255),thickness=3)
			cv2.circle(imgQR,centroid(polygon),2,(0,0,255),3)
			#sos = side of square  
			sos = calculateDistance(polygon[0][0],polygon[0][1],polygon[1][0],polygon[1][1])
			
			#this is disabled due to hardware issues
			#Y = check_y(rectangle)
			#if Y != "Y=0\n\r":
			#print(Y)
			#ser.write(Y)
			#elif Z != "Z=0\n\r":
			Z = check_z(polygon)
			
			if Z == "z=0\n\r":
				Z_set = True
			
			if Z_set == False:
				print(Z)
				ser.write(Z)
			elif X_set == False:
				ser.write("Z+30\n\r")
				ser.write("G=60\n\r")
				sleep(2)
				
			
			X = check_x(sos)
			if Z_set == True and X_set == False:
				print("X="+X)
				ser.write(b"X="+X+b"\n\r")
				await_x_ok = True
				X_set = True
				sleep(2)
				
			if X_set == True:
				print("X is set")
				ser.write("G=115\n\r")
				sleep(2)
				ser.write("X=0\n\r")
				sleep(1)
				ser.write("Z=0\n\r")
				X_set = False
		cv2.imshow('Kamerka', imgQR)
		key = cv2.waitKey(1) & 0xFF
		rawCapture.truncate(0)
		
		if key == ord('q'): break
		
		#waiting for a response from Arduino
		#not working properly
		if ser.in_waiting > 0:
			line = ser.readline()
			if await_x_ok == True and line.find("X OK"):
				X_set=True
				await_x_ok=False
			print(line)
