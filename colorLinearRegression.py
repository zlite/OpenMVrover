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

uart = UART(3, 19200)  # Use UART 3 (pins 4 and 5). No need to go faster than this. Slower = solid comms


THRESHOLD = (0, 100, -35, 127, -128, -30) # blue threshold
# THRESHOLD = (0, 100, 35, 127, -15, 127) # red threshold

TARGET_POINTS = [(70, 60),   # (x, y) CHANGE ME!
                 (285, 60),
                 (319, 239),
                 (1,239)]




cruise_speed = 1575 # how fast should the car drive, range from 1000 to 2000. 1500 is stopped. 1575 is slowest it can go
steering_direction = -1   # use this to revers the steering if your car goes in the wrong direction
steering_gain = 1.1  # calibration for your car's steering sensitivity
steering_center = 80  # set to your car servo's center point
kp = 0.4   # P term of the PID
ki = 0.0     # I term of the PID
kd = 0.3     # D term of the PID

red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
ir_led    = LED(4)
led_state = True
led_time = pyb.millis()

old_error = 0
measured_angle = 0
gain = 5
set_angle = 90 # this is the desired steering angle (straight ahead)
p_term = 0
i_term = 0
d_term = 0
old_time = pyb.millis()
temp = ""

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
    global steering_gain, cruise_speed, steering_center
    angle = int(round((angle+steering_center)*steering_gain))
    constrain(angle, 0, 180)
    ch1 = str(angle)  # must convert to text to send via Serial
    ch2 = str(cruise_speed)  # send throttle data, too
#    print(angle)
    uart.write(ch1)   # send to the Arduino. It looks for channel outputs in order, seperated by a "," and ended with a "\n"
    uart.write(",")
    uart.write(ch2)
    uart.write("\n")

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
    if switch == 1:  # Teensy says you're in MV mode
        img = sensor.snapshot().lens_corr(strength = 1.8, zoom = 1)  # do lens correcton for fisheye
        img = img.rotation_corr(corners = TARGET_POINTS)    # correct perspective to give top-down view
        line = img.get_regression([THRESHOLD], robust = True)  # do linear regression for desired color
        if (line):
            img.draw_line(line.line(), color = 127)
            deflection_angle = -math.atan2(line.line()[1]-line.line()[3],line.line()[0]-line.line()[2])

            # Convert angle in radians to degrees.
            deflection_angle = math.degrees(deflection_angle)
            deflection_angle = 90 - deflection_angle

            print("FPS %f" % clock.fps())
        #    print(deflection_angle)

            # Now you have an angle telling you how much to turn the robot which
            # incorporates the part of the line nearest to the robot and parts of
            # the line farther away from the robot for a better prediction.
        #    print("Turn Angle: %f" % deflection_angle)
            now = pyb.millis()
            if  now > old_time + 1.0 :  # time has passed since last measurement
                if now > led_time + 1000: # switch LED every second
                    if led_state == True:
                        led_control(0) # turn off LED
                        led_state = False
                    else:
                        led_control(2) # turn on LED
                        led_state = True
                    led_time = now   # reset time counter

                measured_angle = deflection_angle + 90
                steer_angle = update_pid()
                old_time = now
                steer (steer_angle*-gain)
#                print(steer_angle*-gain)
    else:
        print(clock.fps())
        time.sleep(0.01)

