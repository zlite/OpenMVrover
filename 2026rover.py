import machine
import time
import csi
import pyb

# Pin definitions - Inputs
CH1_PIN = "P0"  # Throttle input
CH2_PIN = "P3"  # Steering input
CH3_PIN = "P9"  # Mode switch (manual/auto)

# Pin definitions - Outputs
CH1_OUT_PIN = "P8"  # Throttle output (always passthrough)
CH2_OUT_PIN = "P7"  # Steering output (manual passthrough or auto)

# N6 PWM output mapping for the selected output pins
THROTTLE_TIMER_CHANNEL = 2  # P8 = TIM4 CH2
STEERING_TIMER_CHANNEL = 1  # P7 = TIM4 CH1

# Color threshold for blue lane markers
COLOR_THRESHOLDS = [(0, 100, -100, 127, -128, -29)]

# Mode threshold - above 1600us = auto mode, below = manual
MODE_THRESHOLD = 1600

# PID Controller
class PID:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.ticks_ms()
    
    def update(self, error):
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0  # Convert to seconds
        
        if dt <= 0:
            dt = 0.01
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with windup protection
        self.integral += error * dt
        self.integral = max(min(self.integral, 100), -100)
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        
        # Calculate output
        output = p_term + i_term + d_term
        output = max(min(output, self.output_max), self.output_min)
        
        self.prev_error = error
        self.last_time = current_time
        
        return output

class PWMReader:
    def __init__(self, pin_name):
        self.pin = machine.Pin(pin_name, machine.Pin.IN)
        self.pulse_start = 0
        self.pulse_width = 0
        self.last_value = 0
        
        # Set up interrupt for both rising and falling edges
        self.pin.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, 
                     handler=self._irq_handler)
    
    def _irq_handler(self, pin):
        """Interrupt handler called on rising and falling edges"""
        current_time = time.ticks_us()
        current_value = pin.value()
        
        if current_value == 1 and self.last_value == 0:
            # Rising edge - start of pulse
            self.pulse_start = current_time
        elif current_value == 0 and self.last_value == 1:
            # Falling edge - end of pulse
            if self.pulse_start > 0:
                pw = time.ticks_diff(current_time, self.pulse_start)
                # Only accept valid RC PWM range (800-2200 μs)
                if 800 < pw < 2200:
                    self.pulse_width = pw
        
        self.last_value = current_value
    
    def get_pulse_width(self):
        """Get the most recent pulse width measurement"""
        return self.pulse_width

def find_lane_center(img):
    """Find lane markers and calculate center position"""
    try:
        # Find blobs matching blue lane markers
        blobs = img.find_blobs(COLOR_THRESHOLDS, pixels_threshold=100, area_threshold=100, merge=True)
    except Exception as e:
        print(f"ERROR in find_lane_center: {e}")
        return None, []
    
    if not blobs or len(blobs) == 0:
        return None, []
    
    # Sort blobs by x position (left to right)
    sorted_blobs = sorted(blobs, key=lambda b: b.cx())
    
    # Draw bounding boxes and centroids
    for blob in sorted_blobs:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
    
    # If we have at least 2 blobs, find center between leftmost and rightmost
    if len(sorted_blobs) >= 2:
        left_blob = sorted_blobs[0]
        right_blob = sorted_blobs[-1]
        
        # Calculate center position
        lane_center_x = (left_blob.cx() + right_blob.cx()) // 2
        
        # Draw center line
        img.draw_line(lane_center_x, 0, lane_center_x, img.height(), color=(0, 0, 255))
        
        return lane_center_x, sorted_blobs
    
    return None, sorted_blobs

def steering_pwm_from_error(error, image_width, pid):
    """Convert steering error to PWM value using PID"""
    # error is in pixels, normalize to -1 to 1 range using the active image width
    normalized_error = error / (image_width / 2.0)
    
    # PID output is steering adjustment in microseconds
    adjustment = pid.update(normalized_error)
    
    # Center steering is 1500us, apply adjustment
    steering_pwm = 1500 + int(adjustment)
    
    # Clamp to valid servo range
    steering_pwm = max(min(steering_pwm, 2000), 1000)
    
    return steering_pwm

class ExponentialSmoothing:
    """Exponential smoothing filter to reduce jitter"""
    def __init__(self, alpha=0.3):
        self.alpha = alpha  # Smoothing factor (0-1, lower = more smoothing)
        self.value = None
    
    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return int(self.value)

def make_rc_timer(timer_id=4):
    """Create a 50Hz timer where pulse_width values are in microseconds."""
    timer = pyb.Timer(timer_id)
    prescaler = (timer.source_freq() // 1000000) - 1
    timer.init(prescaler=prescaler, period=19999)
    return timer

def main():
    print("Autonomous Lane Following System")
    print("CH1 (P0): Throttle - Always Manual")
    print("CH2 (P3): Steering - Auto/Manual")
    print("CH3 (P9): Mode Switch (<1500=Manual, >1500=Auto)")
    print("-" * 50)
    
    # Initialize LED for heartbeat (1Hz blink)
    led = machine.LED("LED_RED")
    led_state = False
    last_led_toggle = time.ticks_ms()
    
    # Initialize camera
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.QVGA)  # 320x240
    cam.snapshot(time=2000)
    cam.auto_gain(False)
    cam.auto_whitebal(False)
    
    # Initialize PWM readers
    ch1 = PWMReader(CH1_PIN)  # Throttle
    ch2 = PWMReader(CH2_PIN)  # Steering
    ch3 = PWMReader(CH3_PIN)  # Mode
    
    # Initialize smoothing filters for RC inputs
    ch1_smoother = ExponentialSmoothing(alpha=0.2)
    ch2_smoother = ExponentialSmoothing(alpha=0.2)
    ch3_smoother = ExponentialSmoothing(alpha=0.2)
    
    # Initialize PWM outputs (50Hz for RC servos)
    rc_timer = make_rc_timer()
    pwm_throttle = rc_timer.channel(
        THROTTLE_TIMER_CHANNEL,
        pyb.Timer.PWM,
        pin=machine.Pin(CH1_OUT_PIN),
        pulse_width=1500,
    )
    pwm_steering = rc_timer.channel(
        STEERING_TIMER_CHANNEL,
        pyb.Timer.PWM,
        pin=machine.Pin(CH2_OUT_PIN),
        pulse_width=1500,
    )
    
    # Initialize PID controller for steering
    # PID gains: kp, ki, kd, output_min, output_max (in microseconds)
    steering_pid = PID(kp=300, ki=10, kd=50, output_min=-500, output_max=500)
    
    # Initialize smoothing filters for outputs (more aggressive)
    throttle_smoother = ExponentialSmoothing(alpha=0.15)
    steering_smoother = ExponentialSmoothing(alpha=0.1)  # Very smooth for auto steering
    
    # Give interrupts time to stabilize
    time.sleep_ms(100)
    
    while True:
        loop_start = time.ticks_ms()
        
        # Toggle LED every 500ms (1Hz pulse)
        if time.ticks_diff(loop_start, last_led_toggle) >= 500:
            led_state = not led_state
            if led_state:
                led.on()
            else:
                led.off()
            last_led_toggle = loop_start
        
        # Capture image
        img = cam.snapshot()
        
        # Read and smooth RC inputs
        throttle_raw = ch1.get_pulse_width()
        steering_raw = ch2.get_pulse_width()
        mode_raw = ch3.get_pulse_width()
        
        # Apply smoothing to inputs
        throttle_in = ch1_smoother.update(throttle_raw) if throttle_raw > 0 else 0
        steering_in = ch2_smoother.update(steering_raw) if steering_raw > 0 else 0
        mode_in = ch3_smoother.update(mode_raw) if mode_raw > 0 else 0
        
        # Determine mode
        auto_mode = mode_in > MODE_THRESHOLD if mode_in > 0 else False
        
        # Variables to track actual outputs
        throttle_out = throttle_in
        steering_out = steering_in
        
        # Throttle is always passthrough (with smoothing)
        if throttle_in > 0:
            throttle_out = throttle_smoother.update(throttle_in)
            pwm_throttle.pulse_width(throttle_out)
        else:
            throttle_out = 0
        
        # Steering logic
        if auto_mode:
            try:
                # Auto mode - use camera for steering
                lane_center_x, blobs = find_lane_center(img)
                
                if lane_center_x is not None:
                    # Calculate error (desired center is at img.width()//2)
                    image_center = img.width() // 2
                    error = lane_center_x - image_center
                    
                    # Get steering PWM from PID controller
                    steering_pwm = steering_pwm_from_error(error, img.width(), pid=steering_pid)
                    steering_out = steering_smoother.update(steering_pwm)
                    pwm_steering.pulse_width(steering_out)
                    
                    # Display info on image
                    img.draw_string(10, 10, f"AUTO - Error: {error}px", color=(255, 255, 0))
                    img.draw_string(10, 25, f"Steer: {steering_out}us", color=(255, 255, 0))
                else:
                    # No lanes detected - hold center
                    steering_out = steering_smoother.update(1500)
                    pwm_steering.pulse_width(steering_out)
                    img.draw_string(10, 10, "AUTO - NO LANES!", color=(255, 0, 0))
            except Exception as e:
                print(f"ERROR in auto mode: {e}")
                # Failsafe - center steering
                steering_out = 1500
                pwm_steering.pulse_width(steering_out)
                img.draw_string(10, 10, "AUTO - ERROR!", color=(255, 0, 0))
        else:
            # Manual mode - passthrough RC steering
            if steering_in > 0:
                steering_out = steering_smoother.update(steering_in)
                pwm_steering.pulse_width(steering_out)
            img.draw_string(10, 10, "MANUAL MODE", color=(0, 255, 0))
        
        # Display mode and inputs on image
        img.draw_string(10, 40, f"Thr: {throttle_in} Str: {steering_in}", color=(255, 255, 255))
        img.draw_string(10, 55, f"Mode: {mode_in}", color=(255, 255, 255))
        
        # Calculate loop time for debugging
        loop_time = time.ticks_diff(time.ticks_ms(), loop_start)
        
        # Print to serial: inputs, outputs, and timing
        mode_str = "auto" if auto_mode else "manual"
        print(f"IN - Thr:{throttle_in:4d} Str:{steering_in:4d} Mode:{mode_in:4d} | OUT - Thr:{throttle_out:4d} Str:{steering_out:4d} Mode:{mode_str} | Loop:{loop_time}ms")

if __name__ == "__main__":
    main()
