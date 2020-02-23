from dronekit import VehicleMode, connect
from pymavlink import mavutil
import time
from collections import deque
import numpy as np
import argparse
import cv2
import imutils

colors = {
     "red": {"upper": "255, 255, 255",
               "lower":"171, 160, 60"},

     "blue": {"upper": "126, 255, 255",
               "lower":"94, 80, 2"},

     "brown": {"upper": "30,255,255", 
               "lower":"20,100,100"},

     "green": {"upper": "102, 255, 255",
               "lower":"25, 52, 72"},

     "orange": {"upper": "25, 255, 255",
               "lower":"10, 100, 20"},

     "pink": {"upper": "11,255,255",
               "lower":"10,100,100"},

     "black": {"upper": "50,50,100",
               "lower":"0,0,0"},

     "purple": {"upper": "120, 255, 255",
               "lower":"80, 10, 10]"},

     "yellow": {"upper": "44, 255, 255",
               "lower":"24, 100, 100"},

     "white": {"upper": "0,0,255",
               "lower":"0,0,0"},
}


up = colors["blue"]["upper"]
low = colors["blue"]["lower"]
low1 = low.split(",")
up1 = up.split(",")
colorLower = int(low1[0]),int(low1[1]),int(low1[2])
colorUpper = int(up1[0]),int(up1[1]),int(up1[2])

camera = cv2.VideoCapture(0) 



altitude = 4

def initial():
	print ("Checking Armability")
	while not (vehicle.is_armable):
		print ("Arm Olması Bekleniyor...")
		time.sleep(1)

	print ("Robot Arm Edildi" )
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	
	while not (vehicle.armed):
		print ("Arm olması Bekleniyor.")
		time.sleep(1)

def land():
	print ("İnis Yapiliyor...")
	vehicle.mode = VehicleMode("LAND")
	time.sleep(5)
	vehicle.armed= False

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def forward(duration):

	#Kuzey
	if (vehicle.heading >= 0 and vehicle.heading < 22 ) or vehicle.heading >= 337:
		send_ned_velocity(1,0,0,duration)
	#Kuzey doğu
	if vehicle.heading >= 22 or vehicle.heading < 67:
		send_ned_velocity(1,1,0,duration)
	#Doğu
	if vehicle.heading >= 67 and vehicle.heading < 112:
		send_ned_velocity(0,1,0,duration)
	#Güney Doğu
	if vehicle.heading >= 112 and vehicle.heading < 157:
		send_ned_velocity(-1,1,0,duration)
	#Güney
	if vehicle.heading >= 157 and vehicle.heading < 202:
		send_ned_velocity(-1,0,0,duration)
	#Güney Batı
	if vehicle.heading >= 202 and vehicle.heading < 247:
		send_ned_velocity(-1,-1,0,duration)
	#Batı
	if vehicle.heading >= 247 and vehicle.heading < 292:
		send_ned_velocity(0,-1,0,duration)
	#Kuzey batı
	if vehicle.heading > 292 and vehicle.heading < 337:
		send_ned_velocity(1,-1,0,duration)

def back(duration):

	#Kuzey
	if (vehicle.heading >= 0 and vehicle.heading < 22 ) or vehicle.heading >= 337:
		send_ned_velocity(-1,0,0,duration)
	#Kuzey Doğu
	if vehicle.heading >= 22 or vehicle.heading < 67:
		send_ned_velocity(-1,-1,0,duration)
	#Doğu
	if vehicle.heading >= 67 and vehicle.heading < 112:
		send_ned_velocity(0,-1,0,duration)
	#Güney Doğu
	if vehicle.heading >= 112 and vehicle.heading < 157:
		send_ned_velocity(1,-1,0,duration)
	#Güney
	if vehicle.heading >= 157 and vehicle.heading < 202:
		send_ned_velocity(1,0,0,duration)
	#Güney Batı
	if vehicle.heading >= 202 and vehicle.heading < 247:
		send_ned_velocity(1,1,0,duration)
	#Batı
	if vehicle.heading >= 247 and vehicle.heading < 292:
		send_ned_velocity(0,1,0,duration)
	#Kuzey Batı
	if vehicle.heading > 292 and vehicle.heading < 337:
		send_ned_velocity(-1,1,0,duration)

def right(duration):

	#Kuzey
	if (vehicle.heading >= 0 and vehicle.heading < 22 ) or vehicle.heading >= 337:
		send_ned_velocity(0,1,0,duration)
	#Kuzey Doğu
	if vehicle.heading >= 22 or vehicle.heading < 67:
		send_ned_velocity(-1,1,0,duration)
	#Doğu
	if vehicle.heading >= 67 and vehicle.heading < 112:
		send_ned_velocity(-1,0,0,duration)
	#Güney Doğu
	if vehicle.heading >= 112 and vehicle.heading < 157:
		send_ned_velocity(-1,-1,0,duration)
	#Güney
	if vehicle.heading >= 157 and vehicle.heading < 202:
		send_ned_velocity(0,-1,0,duration)
	#Güney Batı
	if vehicle.heading >= 202 and vehicle.heading < 247:
		send_ned_velocity(1,-1,0,duration)
	#Batı
	if vehicle.heading >= 247 and vehicle.heading < 292:
		send_ned_velocity(1,0,0,duration)
	#Kuzey Batı
	if vehicle.heading > 292 and vehicle.heading < 337:
		send_ned_velocity(1,1,0,duration)

def left(duration):
	#Kuzey
	if (vehicle.heading >= 0 and vehicle.heading < 22 ) or vehicle.heading >= 337:
		send_ned_velocity(0,-1,0,duration)
	#Kuzey Doğu
	if vehicle.heading >= 22 or vehicle.heading < 67:
		send_ned_velocity(1,-1,0,duration)
	#Doğu
	if vehicle.heading >= 67 and vehicle.heading < 112:
		send_ned_velocity(1,0,0,duration)
	#Güney Doğu
	if vehicle.heading >= 112 and vehicle.heading < 157:
		send_ned_velocity(1,1,0,duration)
	#Güney
	if vehicle.heading >= 157 and vehicle.heading < 202:
		send_ned_velocity(0,1,0,duration)
	#Güney Batı
	if vehicle.heading >= 202 and vehicle.heading < 247:
		send_ned_velocity(-1,1,0,duration)
	#Batı
	if vehicle.heading >= 247 and vehicle.heading < 292:
		send_ned_velocity(-1,0,0,duration)
	#Kuzey Batı
	if vehicle.heading > 292 and vehicle.heading < 337:
		send_ned_velocity(-1,-1,0,duration)


def takeoff():
	print("Kalkis Yapilyor...")
	vehicle.simple_takeoff(altitude)
	print("Altitude: ", vehicle.location.global_relative_frame.alt)
	if (vehicle.location.global_relative_frame.alt>=altitude*0.95):
		print("Hedeflenen yüksekliğe ulaşıldı...")

def autopilot():
	if(gorev == 1):
		if(x> 110 & x < 210):
			if(y> 80 & y < 150):
				land()
				time.sleep(2)
				gorev = 2
				init = 2 
		elif(x>210):
			if(y < 80):
				forward()
			elif(y > 150):
				back()
			if(y> 80 & x < 110):
				right()

		elif(x > 1 & x < 110 ): 
			if(y < 80):
				forward()
			elif(y > 150):
				back()
			if(y> 80 & x < 110):
				left()
	elif(gorev == 2):
		 if (init ==2):
			 up = colors["red"]["upper"]
			 low = colors["red"]["lower"]
			 up1 = up.split(",")
			 colorLower = int(low1[0]),int(low1[1]),int(low1[2])
			 colorUpper = int(up1[0]),int(up1[1]),int(up1[2])
			 initial()
			 takeoff()
			 time.sleep(2)
			 init =3
		 if(x> 110 & x < 210):
				if(y> 80 & y < 150):
					land()
					time.sleep(2)
					gorev = 3
					init = 4 
		 elif(x>210):
				if(y < 80):
					forward()
				elif(y > 150):
					back()
				if(y> 80 & x < 110):
					right()

		 elif(x > 1 & x < 110 ): 
				if(y < 80):
					forward()
				elif(y > 150):
					back()
				if(y> 80 & x < 110):
					left()
	elif(gorev == 3):
		 if(init == 3):
				up = colors["green"]["upper"]
				low = colors["green"]["lower"]
				low1 = low.split(",")
				up1 = up.split(",")
				colorLower = int(low1[0]),int(low1[1]),int(low1[2])
				colorUpper = int(up1[0]),int(up1[1]),int(up1[2])
				takeoff()
				time.sleep(2)
				init =4
		 if(x> 110 & x < 210):
				if(y> 80 & y < 150):
					land()
					time.sleep(2)
					gorev = 4
					init = 24
		 elif(x>210):
				if(y < 80):
					forward()
				elif(y > 150):
					back()
				if(y> 80 & x < 110):
					right()

		 elif(x > 1 & x < 110 ): 
				if(y < 80):
					forward()
				elif(y > 150):
					back()
				if(y> 80 & x < 110):
					left()

vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
#vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=57600)

initial()
takeoff()

gorev = 1
init = 1
while True: 
 


    if keyboard.is_pressed('d'):
          up = colors["green"]["upper"]
          low = colors["green"]["lower"]
          low1 = low.split(",")
          up1 = up.split(",")

          colorLower = int(low1[0]),int(low1[1]),int(low1[2])
          colorUpper = int(up1[0]),int(up1[1]),int(up1[2])

    (grabbed, frame) = camera.read()


    frame = imutils.resize(frame, width=320, height=240) 
    frame = imutils.rotate(frame, angle=0) 

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=2) 
    mask = cv2.dilate(mask, None, iterations=2) 


    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None


    if len(cnts) > 0:

		     c = max(cnts, key=cv2.contourArea)
		     ((x, y), radius) = cv2.minEnclosingCircle(c)
		     M = cv2.moments(c)
		     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


		     if radius > 10: #algılanacak hedefin minumum boyutu
			     cv2.circle(frame, (int(x), int(y)), int(radius),
				 (0, 255, 255), 2)
			     cv2.circle(frame, center, 5, (0, 0, 255), -1)
    else:
          x = 0
          y = 0
          r = 0
