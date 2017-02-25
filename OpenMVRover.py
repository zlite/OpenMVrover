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

old_error = 0
old_time = 0
measured_angle = 0
p_term = 0
i_term = 0
d_term = 0
old_time = time.time()

def led_control(x):
    if   (x&1)==0: red_led.off()
    elif (x&1)==1: red_led.on()
    if   (x&2)==0: green_led.off()
    elif (x&2)==2: green_led.on()
    if   (x&4)==0: blue_led.off()
    elif (x&4)==4: blue_led.on()
    if   (x&8)==0: ir_led.off()
    elif (x&8)==8: ir_led.on()

def constrain(value, min, max):
    if value < min :
        return min
    if value > max :
        return max
    else:
        return value

def update_pid():
    global old_time, old_error, measured_angle, set_angle
    global p_term, i_term, d_term
    now = time.time()
    dt = now - old_time
    error = set_angle - measured_angle
    de = error - old_error

    p_term = kp * error
    i_term += ki * error
    i_term = constrain(i_term, 0, 100)
    d_term = (de / dt) * kd

    old_error = error
    # print((measured_temp, p_term, i_term, d_term))
    output = p_term + i_term + d_term
    output = constrain(output, 0, 100)
    return output


s1 = Servo(1) # P7 Motor
s2 = Servo(2) # P8 Steering
print (s1.calibration()) # show throttle servo calibration
cruise_speed = 0 # how fast should the car drive, range from 1000 to 2000
steering_gain = 500
kp = 0.8
ki = 0
kd = 0
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565
sensor.set_framesize(sensor.QQVGA) # or sensor.QVGA (or others)
clock = time.clock() # Tracks FPS.
#s1.pulse_width(2000)  # initialize the motor controller with a high signal for a second
#pyb.delay(1000)
#s1.pulse_width(1000) # return motor controller to low/off throttle
while(True):
    led_control(2)
    s1.speed(120)
    pyb.delay(10)
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    img.find_edges(image.EDGE_CANNY, threshold=(40, 174))  # Find edges
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
            now = time.time()
            if  now > old_time + 1 :
                old_time = now
    #            measured_angle = read_angle()
                steer_angle = update_pid()
                steer (steer_angle)
                print(str(measured_angle) + ', ' + str(set_angle) + ', ' + str(steer_angle))
            steer_angle = 1/(totalslope/counter) # 1/(average slope), to compensate for the inverse slope curve
            print (steer_angle)
#           s1.pulse_width(cruise_speed) # move forward at cruise speed
            s2.pulse_width(1500 - int(steer_angle*steering_gain)) # steer
            pyb.delay(10)
