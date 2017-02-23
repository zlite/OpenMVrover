# Autonomous rover code by Chris Anderson, 3DR, DIY Robocars
# Copyrigbt 2017
# Released under a BSD 2.0 Open Source Licence

import sensor, image, time, pyb
from pyb import Servo
from pyb import LED

red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)

def led_control(x):
    if   (x&1)==0: red_led.off()
    elif (x&1)==1: red_led.on()
    if   (x&2)==0: green_led.off()
    elif (x&2)==2: green_led.on()
    if   (x&4)==0: blue_led.off()
    elif (x&4)==4: blue_led.on()
    if   (x&8)==0: ir_led.off()
    elif (x&8)==8: ir_led.on()


s1 = Servo(1) # P7 Motor
s2 = Servo(2) # P8 Steering
print (s1.calibration()) # show throttle servo calibration
cruise_speed = 1500 # how fast should the car drive, range from 1000 to 2000
steering_gain = 800
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565
sensor.set_framesize(sensor.QQVGA) # or sensor.QVGA (or others)
clock = time.clock() # Tracks FPS.
while(True):
    led_control(2)
    s1.pulse_width(cruise_speed) # move forward at cruise speed
#    s1.speed(120)  # alternative way to control speed
    pyb.delay(10)
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    img.find_edges(image.EDGE_CANNY, threshold=(40, 174))  # Find edges
    lines = img.find_lines(threshold=40) # Find lines.
    counter = 0
    totalslope = 0
    if lines:
        for l in lines:
            img.draw_line(l, color=(127)) # Draw lines
            if (l[2]-l[0]) != 0: # don't allow vertical lines (infinite slope)
                slope = 0  # reset slope
                slope = (l[3]-l[1])/(l[2]-l[0])
            if slope <= -0.5 or slope >= 0.5: # ignore lines that are mostly horizontal (slope between -0.5 and 0.5)
                totalslope = totalslope + slope
                counter = counter + 1
        if counter != 0:
            steer_angle = 1/(totalslope/counter) # 1/(average slope), to compensate for the inverse slope curve
            print (steer_angle)
            s2.pulse_width(1500 - int(steer_angle*steering_gain)) # steer
            pyb.delay(10)
