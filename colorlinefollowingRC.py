# Color Line Following Example with PID Steering

#
# For this script to work properly you should point the camera at a line at a
# 45 or so degree angle. Please make sure that only the line is within the
# camera's field of view.

import sensor, image, pyb, math, time
from pyb import Servo
from pyb import LED
from pyb import UART
uart = UART(3, 9600)  # no need to go faster than this. Slower = solid comms

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green things. You may wish to tune them...
# old thresholds = [(30, 100, 15, 127, 15, 127), # generic_red_thresholds
#              (30, 100, -64, -8, -32, 32), # generic_green_thresholds
#              (0, 15, 0, 40, -80, -20)] # generic_blue_thresholds

threshold_index = 0
# 0 for red, 1 for green, 2 for blue

thresholds = [(0, 100, 38, 127, -128, 127), # generic_red_thresholds
              (0, 100, -128, -8, -128, 77), # generic_green_thresholds
              (0, 100, -128, -10, -128, 51)] # generic_blue_thresholds
# You may pass up to 16 thresholds above. However, it's not really possible to segment any
# scene with 16 thresholds before color thresholds start to overlap heavily.


cruise_speed = 0 # how fast should the car drive, range from 1000 to 2000
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
    global steering_gain, cruise_speed, steering_center
    angle = int(round((angle+steering_center)*steering_gain))
    constrain(angle, 0, 180)
    ch1 = str(angle)  # must convert to text to send via Serial
    ch2 = str(cruise_speed)  # send throttle data, too
    print(angle)
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


# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [ # [ROI, weight]
        (46,30,86,44,0.0),
        (30,82,109,36,0.9)
       ]


# Compute the weight divisor (we're computing this so you don't have to make weights add to 1).
weight_sum = 0
for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.

# Camera setup...
clock = time.clock() # Tracks FPS.
sensor.reset() # Initialize the camera sensor.
sensor.__write_reg(0x6B, 0x22)  # switches camera into advanced calibration mode. See this for more: http://forums.openmv.io/viewtopic.php?p=1358#p1358
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.set_auto_gain(True)    # do some calibration at the start
sensor.set_auto_whitebal(True)
sensor.skip_frames(60) # Let new settings take effect.
sensor.set_auto_gain(False)   # now turn off autocalibration before we start color tracking
sensor.set_auto_whitebal(False)



while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    centroid_sum = 0
    for r in ROIS:
        blobs = img.find_blobs([thresholds[threshold_index]], roi=r[0:4], merge=True) # r[0:4] is roi tuple.
        if blobs:
            # Find the index of the blob with the most pixels.
            most_pixels = 0
            largest_blob = 0
            for i in range(len(blobs)):
                if blobs[i].pixels() > most_pixels:
                    most_pixels = blobs[i].pixels()
                    largest_blob = i

            # Draw a rect around the blob.
            img.draw_rectangle(blobs[largest_blob].rect())
            img.draw_cross(blobs[largest_blob].cx(),
                           blobs[largest_blob].cy())

            centroid_sum += blobs[largest_blob].cx() * r[4] # r[4] is the roi weight.

    center_pos = (centroid_sum / weight_sum) # Determine center of line.

    # Convert the center_pos to a deflection angle. We're using a non-linear
    # operation so that the response gets stronger the farther off the line we
    # are. Non-linear operations are good to use on the output of algorithms
    # like this to cause a response "trigger".
    deflection_angle = 0
    # The 80 is from half the X res, the 60 is from half the Y res. The
    # equation below is just computing the angle of a triangle where the
    # opposite side of the triangle is the deviation of the center position
    # from the center and the adjacent side is half the Y res. This limits
    # the angle output to around -45 to 45. (It's not quite -45 and 45).
    deflection_angle = -math.atan((center_pos-80)/60)

    # Convert angle in radians to degrees.
    deflection_angle = math.degrees(deflection_angle)

    # Now you have an angle telling you how much to turn the robot by which
    # incorporates the part of the line nearest to the robot and parts of
    # the line farther away from the robot for a better prediction.
#    print("Turn Angle: %f" % deflection_angle)
    now = pyb.millis()
    if  now > old_time + 0.01 :  # time has passed since last measurement; this will do it at 100Hz
        measured_angle = deflection_angle + 90
        steer_angle = update_pid()
        old_time = now
        steer (steer_angle)
        print(str(measured_angle) + ', ' + str(set_angle) + ', ' + str(steer_angle))
