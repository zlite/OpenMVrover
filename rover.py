# Autonomous rover code by Chris Anderson, 3DR, DIY Robocars
# Copyrigbt 2017
# Released under a BSD 2.0 Open Source Licence

import sensor, image, time, pyb
from pyb import Servo
s1 = Servo(1) # P7 Motor
s2 = Servo(2) # P8 Steering
cruise_speed = 1484 # how fast should the car drive, range from 1000 to 2000
steering_gain = 80
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565
sensor.set_framesize(sensor.QQVGA) # or sensor.QVGA (or others)
clock = time.clock() # Tracks FPS.
#s1.pulse_width(2000)  # initialize the motor controller with a high signal for a second
#pyb.delay(1000)
#s1.pulse_width(1000) # return motor controller to low/off throttle
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    img.find_edges(image.EDGE_CANNY, threshold=(74, 174))  # Find edges
    lines = img.find_lines(threshold=40) # Find lines.
    counter = 0
    totalslope = 0
    for l in lines:
        img.draw_line(l, color=(127)) # Draw lines
    if lines:
        if (l[2]-l[0]) != 0: # don't allow vertical lines (infinite slope)
            slope = 0
            slope = (l[3]-l[1])/(l[2]-l[0])
#            print ('slope')
#            print (slope)
            if slope <= -0.5 or slope >= 0.5: # ignore lines that are mostly horizontal (slope between -0.5 and 0.5)
                totalslope = totalslope + slope
                counter = counter + 1
        if counter != 0:
            steer_angle = 1/(totalslope/counter) # 1/(average slope), to compensate for the inverse slope curve
            print (steer_angle)
            s1.pulse_width(cruise_speed) # move forward at cruise speed
            s2.pulse_width(1500 + int(steer_angle*steering_gain)) # steer
            pyb.delay(5)
