#!/usr/bin/env python3

import itertools
import sys
import time
from sweeppy import Sweep
import math
import serial


angle_offset = 0 # this compensates for the Lidar being placed in a rotated position
start = time.time()
stop = False


PORT_NAME = '/dev/ttyUSB0' # this is for the Lidar
# Teensy_Port = serial.Serial("/dev/ttyACM0", baudrate=19200, timeout=1)

def constrain(val, min_val, max_val):
    if val < min_val: return min_val
    if val > max_val: return max_val
    return val


def scan():
	global stop
	point_list = []
	slope_list = []
	while True:
		print('Recording measurements... Press Crl+C to stop.')
		lasttime = time.time()
		with Sweep(dev) as sweep:
			speed = sweep.get_motor_speed()
			rate = sweep.get_sample_rate()
			print('Motor Speed: {} Hz'.format(speed))
			print('Sample Rate: {} Hz'.format(rate))

	        # Starts scanning as soon as the motor is read
			sweep.start_scanning()

	        # get_scans is coroutine-based generator lazily returning scans ad infinitum        
			for scan in itertools.islice(sweep.get_scans(), 3):
		            print(scan)
			if time.time() < (lasttime + 0.1): 
                # [0] -> New
                # [1] -> Quality
                # [2] -> Angle (0 - 360)
                # [3] -> Distance in mm
				print("time: ", time.time())
                #print("lasttime: ", lasttime)
#	   	if((measurment[3] > 100) and (measurment[3]<2500) and (measurment[1]>2) and ((measurment[2] < 135) or (225 < measurment[2]))):
#                    x_pos = math.cos(math.radians(measurment[2])) * measurment[3] # in mm
#        	    y_pos = math.sin(math.radians(measurment[2])) * measurment[3] # in mm
#           	    point_list.append((x_pos, y_pos))
			else:
				if len(point_list) > 2:
					for i in range(len(point_list)):
						this_point = point_list[i]
						for j in range(len(point_list) - 1 - i):
							other_point = point_list[1 + i]      
							mx = this_point[0] - other_point[0]
							my = this_point[1] - other_point[1]
							temp = math.atan2(my,mx)
							if not math.isnan(temp) and not math.isinf(temp):      
								slope_list.append(temp)         
					slope_list = sorted(slope_list)
					median = math.degrees(slope_list[len(slope_list)/2])
					median = median % 180   # forcing output to be between 0 and 179
					if median < 0: 
						median += 180
					slope_sum = 0
					for j in range(len(slope_list)):
						temp2 = slope_list[j] % 180
						if temp2 < 0:
							temp2 += 180
						temp2 = temp2 - median
						temp2 = temp2*temp2
						slope_sum = slope_sum + temp2
					slope_sum = slope_sum / len(slope_list) 
					slope_sum = math.sqrt(slope_sum)
#               	    	print("Slope sum: ",slope_sum)
					median = 10 * (1/math.tan(math.radians(median)))
#                    		Teensy_Port.write("%f\n" % median)
					print (median)
				point_list = []
				slope_list = []
				lasttime = time.time()

def run():
    '''Main function'''
    time.sleep(1)
    print("Starting...")
    try:
        scan()
    except KeyboardInterrupt:
        stop = True
 
if __name__ == '__main__':
    run
