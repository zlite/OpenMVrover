#!/usr/bin/env python3

import sys
import time
from rplidar import RPLidar
import math
import serial



angle_offset = 0 # this compensates for the Lidar being placed in a rotated position
gain = 1.5 # this is the steering gain. The PWM output to the steering servo must be between 0 (left) and 200 (right)
speed = 1000 # crusing speed, must be between 0 and 3600
steering_correction = -10 # this compensates for any steering bias the car has. Positive numbers steer to the right
start = time.time()
stop = False


PORT_NAME = '/dev/ttyUSB0' # this is for the Lidar
Teensy_Port = serial.Serial("/dev/ttyACM0", baudrate=19200, timeout=1)

def constrain(val, min_val, max_val):
    if val < min_val: return min_val
    if val > max_val: return max_val
    return val


# def steer(angle):
#    angle = 100 + gain*angle
#    angle = int(constrain(angle,0,300))
#    print (angle)


def scan(lidar):
    while True:
        print('Recording measurements... Press Crl+C to stop.')
	counter = 0
	data = 0
        lasttime = time.time()
        for measurment in lidar.iter_measurments():
            if time.time() < (lasttime + 0.1): 
            	if measurment[1] > 2 and ((measurment[2] > 300 or measurment[2] < 60)):  # in angular [2] and quality [1] range
                	if (measurment[3] < 1000 and measurment[3] > 100): # in distance range
#                   	 print (measurment[2])
	                    	if (measurment[2] < 60):   
	                        	temp = measurment[2]
        	            	else:
                	        	temp = -1* (360-measurment[2]) # convert to negative angle to the left of center
                    		data = data + temp # sum of the detected angles, so we can average later
#                   		 range_sum = range_sum + measurment[3] # sum all the distances so we can normalize later
                    		counter = counter + 1 # increment counter
            else:  # 10Hz timer has expired
#                print("this should happen ten times a second")
                if counter > 0:  # this means we see something
                    average_angle = -1 * ((data/counter) - angle_offset) # average of detected angles
                    print ("Average angle: ", average_angle)
		    Teensy_Port.write("%f\n" % average_angle)			
                    obstacle_direction = int(100*math.atan(math.radians(average_angle)))  # convert to a vector component
                    drive_direction = -1 * obstacle_direction # steer in the opposite direction as obstacle (I'll replace this with a PID)
#                    print ("Drive direction: ", drive_direction)
                    counter = 0 # reset counter
                    data = 0  # reset data
                    range_sum = 0
                else:
                    drive_direction = 0
 #               steer(drive_direction)  # Send data to motors
                lasttime = time.time()  # reset 10Hz timer


def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    lidar.start_motor()
    time.sleep(1)
    info = lidar.get_info()
    print(info)
    try:
        scan(lidar)
    except KeyboardInterrupt:
        stop = True
        print('Stopping.')
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
 
if __name__ == '__main__':
    run()
