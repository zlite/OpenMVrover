# Autonomous rover code by Chris Anderson, 3DR, DIY Robocars
# Copyrigbt 2017
# Released under a BSD 2.0 Open Source Licence

import sensor, image, pyb, math, time
from pyb import Servo
from pyb import LED

s1 = Servo(1) # P7 Motor
s2 = Servo(2) # P8 Steering
print (s1.calibration()) # show throttle servo calibration
cruise_speed = 0 # how fast should the car drive, range from 1000 to 2000
steering_gain = 500
kp = 0.8   # P term of the PID
ki = 0     # I term of the PID
kd = 0     # D term of the PID

red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)

old_error = 0
measured_angle = 0
set_angle = 90 # this is the desired steering angle (straight ahead)
p_term = 0
i_term = 0
d_term = 0
old_time = pyb.millis()

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

def steer(angle):
    global steering_gain
    s1.angle(angle) # steer
    pyb.delay(10)

def update_pid():
    global old_time, old_error, measured_angle, set_angle
    global p_term, i_term, d_term
    now = pyb.millis()
    dt = now - old_time
    error = set_angle - measured_angle
    de = error - old_error

    p_term = kp * error
    i_term += ki * error
    i_term = constrain(i_term, 0, 100)
    d_term = (de / dt) * kd

    old_error = error
    output = p_term + i_term + d_term
    output = constrain(output, -50, 50)
    return output

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.RGB565
sensor.set_framesize(sensor.QQVGA) # or sensor.QVGA (or others)
clock = time.clock() # Tracks FPS.
#s1.pulse_width(2000)  # initialize the motor controller with a high signal for a second
#pyb.delay(1000)
#s1.pulse_width(1000) # return motor controller to low/off throttle
while(True):
    led_control(2)
#    s1.speed(120)   Start motor; currently disabled while I use RC for throttle
#    pyb.delay(10)
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    img.find_edges(image.EDGE_CANNY, threshold=(40, 174))  # Find edges
    lines = img.find_lines(threshold=40) # Find lines.
    counter = 0
    totalangle = 0
    for l in lines:
        img.draw_line(l, color=(127)) # Draw lines
    if lines:
        if (l[2]-l[0]) != 0: # don't allow vertical lines (undefined arctan)
            angle = 0
            angle = -math.atan2(-(l[3]-l[1]),(l[2]-l[0])) # arctan (deltaY/deltaX); we use atan2 so we can tell what quadrant it's in
            angle = math.degrees(angle) # convert from radians to degrees
        if angle < 140 and angle > 40: # ignore lines that are mostly horizontal
                totalangle = totalangle + angle  # add up all the line angles so we can average them
                counter = counter + 1
        if counter != 0:
            now = pyb.millis()
            if  now > old_time + 1.0 :
                measured_angle = totalangle/counter  # average of angles
                steer_angle = update_pid()
                old_time = now
                steer (steer_angle)
                print(str(measured_angle) + ', ' + str(set_angle) + ', ' + str(steer_angle))
#           s1.pulse_width(cruise_speed) # move forward at cruise speed; currently disabled while I run throttle with RC

