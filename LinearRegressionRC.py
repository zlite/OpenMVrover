# Color Line Following Example with PID Steering

#
# For this script to work properly you should point the camera at a line at a
# 45 or so degree angle. Please make sure that only the line is within the
# camera's field of view.

import sensor, image, pyb, math, time
from pyb import Servo
from pyb import LED
from pyb import UART
from pyb import Pin


switch_pin = Pin('P7', Pin.IN, Pin.PULL_DOWN)

uart = UART(3, 19200)  # Use UART 3 (pins 4 and 5)

blue_threshold = (0, 100, -128, 127, -128, -35) # L A B

binary_threshold = (57, 255)

rois = [0,40,320,190]

TARGET_POINTS = [(70, 50),   # (x, y), clockwise from top left
                 (285, 50),
                 (319, 239),
                 (1,239)]


cruise_speed = 1575 # how fast should the car drive, range from 1000 (full backwards) to 2000 (full forwards). 1500 is stopped. 1575 is slowest it can go
steering_direction = -1   # use this to revers the steering if your car goes in the wrong direction
steering_gain = 30  # calibration for your car's steering sensitivity
steering_center = -5   # set to your car servo's center point


kp = 0.4   # P term of the PID
ki = 0.0     # I term of the PID
kd = 0.3     # D term of the PID

i_term = 0

old_error = 0
measured_angle = 0
set_angle = 0 # this is the desired steering angle (straight ahead)
old_time = pyb.millis()
led_time = pyb.millis()



RC_control = True  # Set to false if you're just benchtop testing

red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)
led_state = True



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
    angle = int(round((angle+steering_center)*steering_gain))
    ch1 = str(angle)  # must convert to text to send via Serial
    ch2 = str(cruise_speed)  # send throttle data, too
    print("Steering command", angle)
    uart.write(ch1)   # send to the Arduino. It looks for channel outputs in order, seperated by a "," and ended with a "\n"
    uart.write(",")
    uart.write(ch2)
    uart.write("\n")

def update_pid(measured_angle):
    global old_error, i_term
    now = pyb.millis()
    dt = now - old_time
    error = set_angle - measured_angle
    de = error - old_error

    p_term = kp * error
    i_term += ki * error
    i_term = constrain(i_term, 0, 100)
    d_term = (de / dt) * kd

    old_error = error
    output = steering_direction * (p_term + i_term + d_term)
    output = constrain(output, -50, 50)
    return output



# Camera setup...
clock = time.clock() # Tracks FPS.
sensor.reset() # Initialize the camera sensor.
sensor.__write_reg(0x6B, 0x22)  # switches camera into advanced calibration mode. See this for more: http://forums.openmv.io/viewtopic.php?p=1358#p1358
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA) # use QVGA for speed.
sensor.set_vflip(True)
sensor.set_auto_gain(True)    # do some calibration at the start
sensor.set_auto_whitebal(True)
sensor.skip_frames(60) # Let new settings take effect.
sensor.set_auto_gain(False)   # now turn off autocalibration before we start color tracking
sensor.set_auto_whitebal(False)


while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    switch = switch_pin.value() # get value, 0 or 1
    if switch == 1 or  not RC_control:  # Teensy says you're in MV mode
        img = sensor.snapshot().lens_corr(strength = 2.8, zoom = 1)
        img.binary([blue_threshold])
        line = img.get_regression([binary_threshold],roi=rois, robust = False)  # do linear regression for desired color
        if (line):
            img.draw_line(line.line(), color = 127)
            offset = line.line()[2]
            constrain(offset,0,320)
            offset = offset - 160
            offset_angle = math.acos(offset/320)
            offset_angle = math.degrees(offset_angle) - 90
            measured_angle = -math.atan2(line.line()[1]-line.line()[3],line.line()[0]-line.line()[2])
            print(line.line())
            # Convert angle in radians to degrees.
            measured_angle = math.degrees(measured_angle)
            measured_angle = 90 - measured_angle # make straight ahead 0
            measured_angle = constrain(measured_angle, -90, 90)
            net_angle = 2*offset_angle - measured_angle
#            print("Offset Angle: ", offset_angle, "Measured Angle: ", measured_angle, "Net Angle: ", net_angle)

 #           print("FPS %f" % clock.fps())



            now = pyb.millis()
            if  now > old_time + 0.02 :  # time has passed since last measurement; do PID at 50Hz
                if now > led_time + 1000: # switch LED every second
                    if led_state == True:
                        led_control(0) # turn off LED
                        led_state = False
                    else:
                        led_control(2) # turn on Green LED
                        led_state = True
                    led_time = now   # reset time counter
                steer_angle = update_pid(net_angle)
                old_time = now
                steer (steer_angle)

    else:
        now = pyb.millis()
        if  now > old_time + 1.0 :  # time has passed since last measurement
            if now > led_time + 1000: # switch LED every second
                print("Under RC control")
                if led_state == True:
                    led_control(0) # turn off LED
                    led_state = False
                else:
                    led_control(4) # turn on Blue LED
                    led_state = True
                led_time = now   # reset time counter

