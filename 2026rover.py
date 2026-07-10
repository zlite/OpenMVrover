import machine
import time
import csi
import protocol
import network
import socket
import os

# Pin definitions - Inputs
CH1_PIN = "P0"  # Throttle input
CH2_PIN = "P3"  # Steering input
CH3_PIN = "P9"  # Mode switch (manual/auto)

# Pin definitions - Outputs
CH1_OUT_PIN = "P8"  # Throttle output (always passthrough)
CH2_OUT_PIN = "P7"  # Steering output (manual passthrough or auto)

# Color threshold for blue lane markers
COLOR_THRESHOLDS = [(10, 84, -55, 45, -128, -8)]

# Vision performance/tuning
USE_TOP_DOWN_VIEW = True
TOP_DOWN_CORNERS = [
    (115, 80),   # top-left of the lane area in the camera image
    (205, 80),   # top-right
    (319, 239),  # bottom-right
    (0, 239),    # bottom-left
]
LANE_ROI_TOP_PERCENT = 45
BLOB_PIXELS_THRESHOLD = 100
BLOB_AREA_THRESHOLD = 100
BLOB_X_STRIDE = 2
BLOB_Y_STRIDE = 2
SERIAL_PRINT_INTERVAL = 10
DRAW_LANE_FILL = True
DRAW_LANE_CENTERLINE = True
DRAW_BLOB_DEBUG = False
DRAW_OVERLAYS_ONLY_WHEN_USB_CONNECTED = True
USB_CHECK_INTERVAL_MS = 1000
LANE_SAMPLE_SPACING = 20
LANE_SAMPLE_BAND_HEIGHT = 10
LANE_MODEL_STEP = 1
LANE_SMOOTHING_RADIUS = 2
LANE_FILL_MIN_SEPARATION = 20
LANE_FILL_COLOR = (0, 70, 35)
LANE_CENTERLINE_COLOR = (255, 140, 0)
LANE_CENTERLINE_THICKNESS = 3

# Race behavior/tuning
LOOKAHEAD_Y_PERCENT = 70
LOOKAHEAD_Y_FAST_PERCENT = 55
HEADING_GAIN = 0.65
CURVE_FEEDFORWARD_GAIN = 0.20
LANE_CENTER_SMOOTHING = 0.35
AUTO_THROTTLE_ENABLED = True
THROTTLE_NEUTRAL = 1500
AUTO_THROTTLE_MAX = 1580
AUTO_THROTTLE_MIN = 1500
AUTO_THROTTLE_ERROR_SLOWDOWN = 70
AUTO_THROTTLE_CURVE_SLOWDOWN = 70
AUTO_THROTTLE_CONFIDENCE_SLOWDOWN = 80
LANE_HOLD_LAST_MS = 250
LANE_CENTER_FAILSAFE_MS = 700

# Mode threshold - above 1600us = auto mode, below = manual
MODE_THRESHOLD = 1600

# WiFi status page
WIFI_STREAM_ENABLED = True
WIFI_SSID = "AndersonHub"
WIFI_PASSWORD = "momrules"
WIFI_STREAM_PORT = 8080
WIFI_CONNECT_TIMEOUT_MS = 15000
WIFI_CLIENT_TIMEOUT_SEC = 5
WIFI_STATUS_INTERVAL_MS = 250

CONFIG_FIELDS = (
    "LOOKAHEAD_Y_PERCENT",
    "LOOKAHEAD_Y_FAST_PERCENT",
    "HEADING_GAIN",
    "CURVE_FEEDFORWARD_GAIN",
    "LANE_CENTER_SMOOTHING",
    "AUTO_THROTTLE_ENABLED",
    "AUTO_THROTTLE_MAX",
    "AUTO_THROTTLE_MIN",
    "AUTO_THROTTLE_ERROR_SLOWDOWN",
    "AUTO_THROTTLE_CURVE_SLOWDOWN",
    "AUTO_THROTTLE_CONFIDENCE_SLOWDOWN",
    "LANE_HOLD_LAST_MS",
    "LANE_CENTER_FAILSAFE_MS",
    "USE_TOP_DOWN_VIEW",
    "LANE_ROI_TOP_PERCENT",
    "LANE_SAMPLE_SPACING",
    "LANE_SMOOTHING_RADIUS",
    "DRAW_LANE_FILL",
    "DRAW_LANE_CENTERLINE",
    "DRAW_OVERLAYS_ONLY_WHEN_USB_CONNECTED",
)
CONFIG_FILE = "rover_config.py"
CONFIG_TEMP_FILE = "rover_config.tmp"

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
        self.filtered_derivative = 0
        self.derivative_alpha = 0.25
        self.has_previous_error = False
        self.last_time = time.ticks_ms()

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.filtered_derivative = 0
        self.has_previous_error = False
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
        raw_derivative = (error - self.prev_error) / dt if self.has_previous_error else 0
        self.filtered_derivative += self.derivative_alpha * (
            raw_derivative - self.filtered_derivative
        )
        d_term = self.kd * self.filtered_derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        output = max(min(output, self.output_max), self.output_min)
        
        self.prev_error = error
        self.has_previous_error = True
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

    def deinit(self):
        """Disable pin interrupts before shutdown."""
        try:
            self.pin.irq(handler=None)
        except Exception:
            pass

def lane_roi(img, config=None):
    """Return the lower image area where lane markers are expected."""
    roi_top = LANE_ROI_TOP_PERCENT if config is None else config_value(config, "LANE_ROI_TOP_PERCENT")
    y = (img.height() * roi_top) // 100
    return (0, y, img.width(), img.height() - y)

def apply_top_down_view(img, config=None):
    """Apply a calibrated bird's-eye transform when enabled."""
    use_top_down = USE_TOP_DOWN_VIEW if config is None else config_value(config, "USE_TOP_DOWN_VIEW")
    if not use_top_down:
        return img
    try:
        corrected = img.rotation_corr(corners=TOP_DOWN_CORNERS)
        return corrected if corrected else img
    except Exception as e:
        print(f"ERROR in top-down transform: {e}")
        return img

def blob_center_x(blob):
    return blob.cx()

def sample_lane_points(img, config=None):
    """Sample left/right lane marker positions at fixed distance rows."""
    _, roi_y, roi_w, roi_h = lane_roi(img, config)
    sample_band_height = LANE_SAMPLE_BAND_HEIGHT
    sample_spacing = LANE_SAMPLE_SPACING if config is None else config_value(config, "LANE_SAMPLE_SPACING")
    left_points = []
    right_points = []
    y = roi_y + (sample_band_height // 2)
    
    while y < roi_y + roi_h:
        band_y = max(roi_y, y - (sample_band_height // 2))
        band_h = min(sample_band_height, roi_y + roi_h - band_y)
        try:
            band_blobs = img.find_blobs(
                COLOR_THRESHOLDS,
                roi=(0, band_y, roi_w, band_h),
                pixels_threshold=max(10, BLOB_PIXELS_THRESHOLD // 4),
                area_threshold=max(10, BLOB_AREA_THRESHOLD // 4),
                x_stride=BLOB_X_STRIDE,
                y_stride=1,
                merge=True,
            )
        except Exception as e:
            print(f"ERROR in sample_lane_points: {e}")
            return left_points, right_points
        
        if band_blobs and len(band_blobs) >= 2:
            sorted_band_blobs = sorted(band_blobs, key=blob_center_x)
            left_blob = sorted_band_blobs[0]
            right_blob = sorted_band_blobs[-1]
            left_rect = left_blob.rect()
            right_rect = right_blob.rect()
            left_inner_x = left_rect[0] + left_rect[2]
            right_inner_x = right_rect[0]
            
            if right_inner_x - left_inner_x >= LANE_FILL_MIN_SEPARATION:
                # Use the inner edges of each lane marker, not their centers.
                left_points.append((left_inner_x, y))
                right_points.append((right_inner_x, y))
        
        y += sample_spacing
    
    return left_points, right_points

def smooth_points(points, config=None):
    """Apply a spatial moving average to reduce jitter before interpolation."""
    if len(points) < 3:
        return points
    
    smoothing_radius = LANE_SMOOTHING_RADIUS if config is None else config_value(config, "LANE_SMOOTHING_RADIUS")
    smoothed = []
    for i in range(len(points)):
        x_sum = 0
        count = 0
        start = max(0, i - smoothing_radius)
        end = min(len(points) - 1, i + smoothing_radius)
        for j in range(start, end + 1):
            x_sum += points[j][0]
            count += 1
        smoothed.append((x_sum // count, points[i][1]))
    return smoothed

def interpolated_x(points, y):
    """Linear interpolation of x for a given y. This avoids spline overshoot wiggle."""
    if len(points) == 1:
        return points[0][0]
    
    if y <= points[0][1]:
        return points[0][0]
    if y >= points[-1][1]:
        return points[-1][0]
    
    for i in range(len(points) - 1):
        if points[i][1] <= y <= points[i + 1][1]:
            p1 = points[i]
            p2 = points[i + 1]
            break
    else:
        return points[-1][0]
    
    span = p2[1] - p1[1]
    if span <= 0:
        return p1[0]
    
    t = (y - p1[1]) / span
    return int(p1[0] + ((p2[0] - p1[0]) * t))

def draw_lane_model(img, left_points, right_points, draw_overlay, config=None):
    """Draw the lane polygon and curved centerline from sampled marker edges."""
    if len(left_points) < 2 or len(right_points) < 2:
        return []
    
    left_points = smooth_points(left_points, config)
    right_points = smooth_points(right_points, config)
    y_start = max(left_points[0][1], right_points[0][1])
    y_end = min(left_points[-1][1], right_points[-1][1])
    center_points = []
    
    for y in range(y_start, y_end + 1, LANE_MODEL_STEP):
        left_x = interpolated_x(left_points, y)
        right_x = interpolated_x(right_points, y)
        
        if right_x - left_x >= LANE_FILL_MIN_SEPARATION:
            if draw_overlay and config_value(config, "DRAW_LANE_FILL"):
                img.draw_line((left_x, y, right_x, y), color=LANE_FILL_COLOR)
            center_points.append(((left_x + right_x) // 2, y))
    
    if draw_overlay and config_value(config, "DRAW_LANE_CENTERLINE"):
        for i in range(1, len(center_points)):
            img.draw_line(
                (
                    center_points[i - 1][0],
                    center_points[i - 1][1],
                    center_points[i][0],
                    center_points[i][1],
                ),
                color=LANE_CENTERLINE_COLOR,
                thickness=LANE_CENTERLINE_THICKNESS,
            )
    
    return center_points

def expected_lane_samples(img, config=None):
    _, _, _, roi_h = lane_roi(img, config)
    sample_spacing = LANE_SAMPLE_SPACING if config is None else config_value(config, "LANE_SAMPLE_SPACING")
    return max(1, roi_h // sample_spacing)

def lane_width_stats(left_points, right_points):
    count = min(len(left_points), len(right_points))
    if count == 0:
        return 0, 0
    
    width_sum = 0
    min_width = 10000
    max_width = 0
    for i in range(count):
        width = right_points[i][0] - left_points[i][0]
        width_sum += width
        min_width = min(min_width, width)
        max_width = max(max_width, width)
    
    avg_width = width_sum // count
    width_spread = max_width - min_width
    return avg_width, width_spread

def nearest_center_point(center_points, target_y):
    if not center_points:
        return None
    
    best = center_points[0]
    best_dist = abs(center_points[0][1] - target_y)
    for point in center_points:
        dist = abs(point[1] - target_y)
        if dist < best_dist:
            best = point
            best_dist = dist
    return best

def estimate_lane_confidence(img, blobs, left_points, right_points, center_points, config=None):
    expected = expected_lane_samples(img, config)
    sample_count = min(len(left_points), len(right_points))
    sample_score = min(1.0, sample_count / expected)
    blob_score = min(1.0, len(blobs) / 4.0) if blobs else 0.0
    avg_width, width_spread = lane_width_stats(left_points, right_points)
    
    if avg_width > 0:
        width_score = max(0.0, 1.0 - (width_spread / max(avg_width, 1)))
    else:
        width_score = 0.0
    
    if center_points:
        model_score = 1.0
    else:
        model_score = 0.0
    
    confidence = (sample_score * 0.45) + (blob_score * 0.20) + (width_score * 0.25) + (model_score * 0.10)
    return min(1.0, max(0.0, confidence)), avg_width

def estimate_curve(center_points):
    if len(center_points) < 2:
        return 0
    
    near_x = center_points[-1][0]
    far_x = center_points[0][0]
    return near_x - far_x

def dynamic_lookahead_percent(throttle_us, config=None):
    """Look farther up the track as speed demand increases."""
    slow = config_value(config, "LOOKAHEAD_Y_PERCENT")
    fast = config_value(config, "LOOKAHEAD_Y_FAST_PERCENT")
    if throttle_us <= THROTTLE_NEUTRAL:
        return slow
    throttle_max = config_value(config, "AUTO_THROTTLE_MAX")
    speed_ratio = min(1.0, (throttle_us - THROTTLE_NEUTRAL) / max(1, throttle_max - THROTTLE_NEUTRAL))
    return int(slow + ((fast - slow) * speed_ratio))

def path_tracking_error(img, center_points, lookahead_x, config=None):
    """Combine cross-track, heading, and curvature into one pixel error."""
    image_center = img.width() / 2.0
    cross_track = lookahead_x - image_center
    if len(center_points) < 2:
        return cross_track, 0.0

    near_x = center_points[-1][0]
    heading_error = lookahead_x - near_x
    curve_error = center_points[0][0] - near_x
    combined = cross_track
    combined += config_value(config, "HEADING_GAIN") * heading_error
    combined += config_value(config, "CURVE_FEEDFORWARD_GAIN") * curve_error
    return combined, heading_error

def find_lane_center(img, draw_overlay=True, config=None, lookahead_percent=None):
    """Find lane markers and calculate center position"""
    roi = lane_roi(img, config)
    left_points, right_points = sample_lane_points(img, config)
    try:
        # Find blobs matching blue lane markers
        blobs = img.find_blobs(
            COLOR_THRESHOLDS,
            roi=roi,
            pixels_threshold=BLOB_PIXELS_THRESHOLD,
            area_threshold=BLOB_AREA_THRESHOLD,
            x_stride=BLOB_X_STRIDE,
            y_stride=BLOB_Y_STRIDE,
            merge=True,
        )
    except Exception as e:
        print(f"ERROR in find_lane_center: {e}")
        return None, [], {}

    center_points = draw_lane_model(img, left_points, right_points, draw_overlay, config)
    sorted_blobs = sorted(blobs, key=blob_center_x) if blobs else []
    confidence, lane_width = estimate_lane_confidence(img, sorted_blobs, left_points, right_points, center_points, config)
    curve = estimate_curve(center_points)
    if lookahead_percent is None:
        lookahead_percent = config_value(config, "LOOKAHEAD_Y_PERCENT")
    target_y = (img.height() * lookahead_percent) // 100
    lookahead_point = nearest_center_point(center_points, target_y)
    lane_info = {
        "center_points": center_points,
        "sample_count": min(len(left_points), len(right_points)),
        "expected_samples": expected_lane_samples(img, config),
        "confidence": confidence,
        "curve": curve,
        "lane_width": lane_width,
        "lookahead_y": target_y,
    }
    
    if draw_overlay and DRAW_BLOB_DEBUG:
        for blob in sorted_blobs:
            img.draw_rectangle(blob.rect(), color=(255, 0, 0))
            img.draw_cross((blob.cx(), blob.cy()), color=(0, 255, 0))
    
    # Prefer the bottom lane-fill slices for steering because they follow curves.
    if lookahead_point:
        lane_info["lookahead_x"] = lookahead_point[0]
        return lookahead_point[0], sorted_blobs, lane_info
    
    # If we have at least 2 blobs, find center between leftmost and rightmost
    if len(sorted_blobs) >= 2:
        left_blob = sorted_blobs[0]
        right_blob = sorted_blobs[-1]
        
        # Calculate center position
        lane_center_x = (left_blob.cx() + right_blob.cx()) // 2
        
        # Draw center line
        if draw_overlay:
            img.draw_line(
                (lane_center_x, 0, lane_center_x, img.height()),
                color=LANE_CENTERLINE_COLOR,
                thickness=LANE_CENTERLINE_THICKNESS,
            )
        
        lane_info["lookahead_x"] = lane_center_x
        return lane_center_x, sorted_blobs, lane_info
    
    lane_info["lookahead_x"] = None
    return None, sorted_blobs, lane_info

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

def auto_throttle_limit(throttle_in, lane_error, curve, confidence, config):
    """Limit forward throttle based on lane difficulty without adding throttle."""
    auto_throttle_enabled = config_value(config, "AUTO_THROTTLE_ENABLED")
    throttle_neutral = THROTTLE_NEUTRAL
    if not auto_throttle_enabled or throttle_in <= throttle_neutral:
        return throttle_in
    
    error_ratio = min(1.0, abs(lane_error) / 120.0)
    curve_ratio = min(1.0, abs(curve) / 120.0)
    confidence_penalty = 1.0 - max(0.0, min(1.0, confidence))
    
    auto_max = config_value(config, "AUTO_THROTTLE_MAX")
    auto_min = config_value(config, "AUTO_THROTTLE_MIN")
    max_throttle = auto_max
    max_throttle -= int(config_value(config, "AUTO_THROTTLE_ERROR_SLOWDOWN") * error_ratio)
    max_throttle -= int(config_value(config, "AUTO_THROTTLE_CURVE_SLOWDOWN") * curve_ratio)
    max_throttle -= int(config_value(config, "AUTO_THROTTLE_CONFIDENCE_SLOWDOWN") * confidence_penalty)
    max_throttle = max(auto_min, min(auto_max, max_throttle))
    
    return min(throttle_in, max_throttle)

def make_runtime_config():
    config = {}
    for name in CONFIG_FIELDS:
        config[name] = globals()[name]
    load_saved_config(config)
    config["_save_message"] = ""
    return config

def load_saved_config(config):
    try:
        import rover_config
        saved = rover_config.CONFIG
        for name in CONFIG_FIELDS:
            if name in saved:
                config[name] = saved[name]
        config["_saved"] = True
    except Exception:
        config["_saved"] = False

def save_runtime_config(config, allow_write=True):
    if not allow_write:
        return False, "refusing to save while USB is connected"
    
    try:
        with open(CONFIG_TEMP_FILE, "w") as file:
            file.write("# Auto-generated by 2026rover.py calibration page\n")
            file.write("CONFIG = {\n")
            for name in CONFIG_FIELDS:
                file.write("    " + repr(name) + ": " + repr(config[name]) + ",\n")
            file.write("}\n")
            try:
                file.flush()
            except Exception:
                pass
        
        try:
            os.sync()
        except Exception:
            pass
        
        try:
            os.remove(CONFIG_FILE)
        except OSError:
            pass
        os.rename(CONFIG_TEMP_FILE, CONFIG_FILE)
        
        try:
            os.sync()
        except Exception:
            pass
        
        config["_saved"] = True
        config["_save_message"] = "saved"
        return True, "saved"
    except Exception as e:
        config["_save_message"] = str(e)
        return False, str(e)

def reset_saved_config(config):
    try:
        os.remove(CONFIG_FILE)
    except OSError:
        pass
    try:
        os.remove(CONFIG_TEMP_FILE)
    except OSError:
        pass
    for name in CONFIG_FIELDS:
        config[name] = globals()[name]
    config["_saved"] = False
    config["_save_message"] = "reset"
    return True, "reset"

def config_value(config, name):
    if config is None:
        return globals()[name]
    return config.get(name, globals()[name])

def host_connected():
    """Return whether OpenMV IDE/Protocol V2 has an active host connection."""
    try:
        return protocol.is_active()
    except Exception:
        return False

def bool_text(value):
    return "true" if value else "false"

def parse_bool(value):
    return value in ("1", "true", "True", "on", "yes")

def parse_query_string(path):
    query = {}
    parts = path.split(b"?", 1)
    if len(parts) < 2:
        return query
    
    for item in parts[1].split(b"&"):
        if b"=" in item:
            key, value = item.split(b"=", 1)
            try:
                query[key.decode()] = value.decode()
            except Exception:
                pass
    return query

def apply_config_query(config, query):
    changed = False
    for name in CONFIG_FIELDS:
        if name not in query:
            continue
        
        current = config[name]
        raw = query[name]
        try:
            if isinstance(current, bool):
                config[name] = parse_bool(raw)
            elif isinstance(current, int):
                config[name] = int(float(raw))
            else:
                config[name] = float(raw)
            changed = True
        except Exception:
            pass
    return changed

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

class ServoPWM:
    """Firmware 5.0/N6 servo PWM using the portable machine API."""
    def __init__(self, pin_name, initial_us=1500):
        self.pwm = machine.PWM(
            machine.Pin(pin_name),
            freq=50,
            duty_ns=int(initial_us * 1000),
        )

    def pulse_width(self, pulse_us):
        self.pwm.duty_ns(int(pulse_us * 1000))

    def deinit(self):
        self.pwm.deinit()

def wlan_station_id():
    try:
        return network.WLAN.IF_STA
    except AttributeError:
        return network.STA_IF

def start_wifi_stream():
    """Start a simple browser-compatible status server."""
    if not WIFI_STREAM_ENABLED:
        return None
    
    try:
        wlan = network.WLAN(wlan_station_id())
        wlan.active(True)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        
        connect_start = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), connect_start) > WIFI_CONNECT_TIMEOUT_MS:
                print("WiFi stream disabled: connection timed out")
                return None
            print("Connecting to WiFi...")
            time.sleep_ms(500)
        
        addr = wlan.ifconfig()[0]
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", WIFI_STREAM_PORT))
        server.listen(1)
        server.settimeout(0)
        print(f"WiFi status: http://{addr}:{WIFI_STREAM_PORT}/")
        return {
            "server": server,
            "client": None,
            "last_status": "Starting...",
        }
    except Exception as e:
        print(f"WiFi stream disabled: {e}")
        return None

def close_stream_client(stream):
    try:
        stream["client"].close()
    except Exception:
        pass
    stream["client"] = None

def socket_send_all(sock, data):
    if isinstance(data, str):
        data = data.encode()
    
    try:
        total = len(data)
    except TypeError:
        total = data.size()
    
    sent = 0
    while sent < total:
        chunk = data[sent:min(sent + 1024, total)]
        written = sock.send(chunk)
        if written is None:
            written = len(chunk)
        sent += written

def read_http_path(request):
    try:
        first_line = request.split(b"\r\n", 1)[0]
        parts = first_line.split()
        if len(parts) >= 2:
            return parts[1]
    except Exception:
        pass
    return b"/"

def send_status_page(client):
    page = (
        "<!doctype html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>OpenMV Rover</title>"
        "<style>body{margin:0;background:#111;color:#eee;font:16px monospace;padding:16px}"
        "pre{white-space:pre-wrap;line-height:1.45;margin:0}</style>"
        "<script>"
        "function tick(){"
        "fetch('/status').then(function(r){return r.text()})"
        ".then(function(t){document.getElementById('s').textContent=t});"
        "}"
        "window.onload=tick;"
        "setInterval(tick," + str(WIFI_STATUS_INTERVAL_MS) + ");"
        "</script>"
        "</head><body><p><a href='/calibrate' style='color:#8cf'>Calibration</a></p><pre id='s'>Connecting...</pre></body></html>"
    )
    socket_send_all(
        client,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Length: " + str(len(page)) + "\r\n\r\n" + page
    )

def config_text(config):
    lines = ["SAVED=" + str(config.get("_saved", False))]
    for name in CONFIG_FIELDS:
        lines.append(f"{name}={config[name]}")
    return "\n".join(lines)

def calibration_field(name, label, value, kind="number", step="1"):
    if isinstance(value, bool):
        checked = " checked" if value else ""
        return (
            "<label><span>" + label + "</span>"
            "<input type='checkbox' name='" + name + "' value='1'" + checked + "></label>"
        )
    return (
        "<label><span>" + label + "</span>"
        "<input type='" + kind + "' step='" + step + "' name='" + name + "' value='" + str(value) + "'></label>"
    )

def send_calibration_page(client, config):
    page = (
        "<!doctype html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Rover Calibration</title>"
        "<style>body{margin:0;background:#111;color:#eee;font:15px sans-serif;padding:16px}"
        "form{display:grid;gap:10px;max-width:560px}"
        "label{display:grid;grid-template-columns:1fr 120px;gap:12px;align-items:center}"
        "input{font:15px sans-serif;padding:6px;background:#222;color:#eee;border:1px solid #555}"
        "input[type=checkbox]{width:24px;height:24px}button,a{color:#8cf;font:15px sans-serif}</style>"
        "</head><body><h2>Rover Calibration</h2>"
        "<p>Config source: " + ("saved" if config.get("_saved", False) else "defaults") + "</p>"
        "<p>Save status: " + str(config.get("_save_message", "")) + "</p>"
        "<form action='/set' method='get'>"
        + calibration_field("LOOKAHEAD_Y_PERCENT", "Lookahead Y %", config["LOOKAHEAD_Y_PERCENT"])
        + calibration_field("LOOKAHEAD_Y_FAST_PERCENT", "Fast lookahead Y %", config["LOOKAHEAD_Y_FAST_PERCENT"])
        + calibration_field("HEADING_GAIN", "Heading gain", config["HEADING_GAIN"], step="0.05")
        + calibration_field("CURVE_FEEDFORWARD_GAIN", "Curve feed-forward", config["CURVE_FEEDFORWARD_GAIN"], step="0.05")
        + calibration_field("LANE_CENTER_SMOOTHING", "Lane center smoothing", config["LANE_CENTER_SMOOTHING"], step="0.05")
        + calibration_field("AUTO_THROTTLE_ENABLED", "Auto throttle", config["AUTO_THROTTLE_ENABLED"])
        + calibration_field("AUTO_THROTTLE_MAX", "Auto throttle max", config["AUTO_THROTTLE_MAX"])
        + calibration_field("AUTO_THROTTLE_MIN", "Auto throttle min", config["AUTO_THROTTLE_MIN"])
        + calibration_field("AUTO_THROTTLE_ERROR_SLOWDOWN", "Error slowdown", config["AUTO_THROTTLE_ERROR_SLOWDOWN"])
        + calibration_field("AUTO_THROTTLE_CURVE_SLOWDOWN", "Curve slowdown", config["AUTO_THROTTLE_CURVE_SLOWDOWN"])
        + calibration_field("AUTO_THROTTLE_CONFIDENCE_SLOWDOWN", "Confidence slowdown", config["AUTO_THROTTLE_CONFIDENCE_SLOWDOWN"])
        + calibration_field("LANE_HOLD_LAST_MS", "Hold last steering ms", config["LANE_HOLD_LAST_MS"])
        + calibration_field("LANE_CENTER_FAILSAFE_MS", "Center failsafe ms", config["LANE_CENTER_FAILSAFE_MS"])
        + calibration_field("USE_TOP_DOWN_VIEW", "Top-down transform", config["USE_TOP_DOWN_VIEW"])
        + calibration_field("LANE_ROI_TOP_PERCENT", "ROI top %", config["LANE_ROI_TOP_PERCENT"])
        + calibration_field("LANE_SAMPLE_SPACING", "Lane sample spacing", config["LANE_SAMPLE_SPACING"])
        + calibration_field("LANE_SMOOTHING_RADIUS", "Lane smoothing radius", config["LANE_SMOOTHING_RADIUS"])
        + calibration_field("DRAW_LANE_FILL", "Draw lane fill", config["DRAW_LANE_FILL"])
        + calibration_field("DRAW_LANE_CENTERLINE", "Draw centerline", config["DRAW_LANE_CENTERLINE"])
        + calibration_field("DRAW_OVERLAYS_ONLY_WHEN_USB_CONNECTED", "Overlays only on USB", config["DRAW_OVERLAYS_ONLY_WHEN_USB_CONNECTED"])
        + "<button type='submit'>Apply</button></form>"
        "<form action='/save' method='get' style='margin-top:14px'><button type='submit'>Save to chip</button></form>"
        "<form action='/reset' method='get' style='margin-top:8px'><button type='submit'>Reset saved config</button></form>"
        "<p><a href='/'>Telemetry</a> | <a href='/config'>Raw config</a></p></body></html>"
    )
    socket_send_all(
        client,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "Content-Length: " + str(len(page)) + "\r\n\r\n" + page
    )

def update_wifi_status(stream, status, config, usb_connected=False):
    """Serve the latest status text over WiFi."""
    if stream is None:
        return
    
    stream["last_status"] = status
    
    try:
        client, _ = stream["server"].accept()
        client.settimeout(WIFI_CLIENT_TIMEOUT_SEC)
        try:
            request = client.recv(1024)
        except Exception:
            request = b""
        
        path = read_http_path(request)
        route = path.split(b"?", 1)[0]
        if route == b"/status":
            socket_send_all(
                client,
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n"
            )
            socket_send_all(client, stream["last_status"])
        elif route == b"/calibrate":
            send_calibration_page(client, config)
        elif route == b"/config":
            socket_send_all(
                client,
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n"
            )
            socket_send_all(client, config_text(config))
        elif route == b"/set":
            query = parse_query_string(path)
            for name in CONFIG_FIELDS:
                if isinstance(config[name], bool) and name not in query:
                    query[name] = "0"
            apply_config_query(config, query)
            config["_saved"] = False
            socket_send_all(
                client,
                "HTTP/1.1 303 See Other\r\n"
                "Location: /calibrate\r\n"
                "Connection: close\r\n\r\n"
            )
        elif route == b"/save":
            save_runtime_config(config, allow_write=not usb_connected)
            socket_send_all(
                client,
                "HTTP/1.1 303 See Other\r\n"
                "Location: /calibrate\r\n"
                "Connection: close\r\n\r\n"
            )
        elif route == b"/reset":
            if usb_connected:
                config["_save_message"] = "refusing to reset saved config while USB is connected"
            else:
                reset_saved_config(config)
            socket_send_all(
                client,
                "HTTP/1.1 303 See Other\r\n"
                "Location: /calibrate\r\n"
                "Connection: close\r\n\r\n"
            )
        else:
            send_status_page(client)
        client.close()
    except OSError:
        return
    except Exception as e:
        print(f"WiFi status error: {e}")

def stop_wifi_stream(stream):
    if stream is None:
        return
    close_stream_client(stream)
    try:
        stream["server"].close()
    except Exception:
        pass

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
    usb_connected = host_connected()
    last_usb_check = time.ticks_ms()
    
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
    
    # Initialize firmware 5.0 PWM outputs (50Hz for RC servos).
    pwm_throttle = ServoPWM(CH1_OUT_PIN)
    pwm_steering = ServoPWM(CH2_OUT_PIN)
    
    # Initialize PID controller for steering
    # PID gains: kp, ki, kd, output_min, output_max (in microseconds)
    steering_pid = PID(kp=300, ki=10, kd=50, output_min=-500, output_max=500)
    
    # Initialize smoothing filters for outputs (more aggressive)
    config = make_runtime_config()
    throttle_smoother = ExponentialSmoothing(alpha=0.15)
    steering_smoother = ExponentialSmoothing(alpha=0.1)  # Very smooth for auto steering
    lane_center_smoother = ExponentialSmoothing(alpha=config_value(config, "LANE_CENTER_SMOOTHING"))
    wifi_stream = start_wifi_stream()
    
    # Give interrupts time to stabilize
    time.sleep_ms(100)
    fps = 0.0
    frame_count = 0
    min_fps = 999.0
    max_fps = 0.0
    lane_lost_count = 0
    lane_was_detected = False
    auto_was_enabled = False
    last_lane_seen_ms = time.ticks_ms()
    last_auto_steering = 1500
    
    try:
        while True:
            frame_count += 1
            loop_start = time.ticks_ms()
        
            # Toggle LED every 500ms (1Hz pulse)
            if time.ticks_diff(loop_start, last_led_toggle) >= 500:
                led_state = not led_state
                if led_state:
                    led.on()
                else:
                    led.off()
                last_led_toggle = loop_start
            
            if time.ticks_diff(loop_start, last_usb_check) >= USB_CHECK_INTERVAL_MS:
                usb_connected = host_connected()
                last_usb_check = loop_start
            
            overlay_enabled = usb_connected or not config_value(config, "DRAW_OVERLAYS_ONLY_WHEN_USB_CONNECTED")
            
            # Capture image
            img = cam.snapshot()
            img = apply_top_down_view(img, config)
            
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
            lane_center_smoother.alpha = config_value(config, "LANE_CENTER_SMOOTHING")
            lookahead_percent = dynamic_lookahead_percent(throttle_in, config)
            lane_center_x, blobs, lane_info = find_lane_center(
                img,
                draw_overlay=overlay_enabled,
                config=config,
                lookahead_percent=lookahead_percent,
            )
            image_center = img.width() // 2
            lane_error = 0
            heading_error = 0
            lane_detected = lane_center_x is not None
            if lane_detected:
                lane_center_x = lane_center_smoother.update(lane_center_x)
                lane_error, heading_error = path_tracking_error(
                    img, lane_info.get("center_points", []), lane_center_x, config
                )
                last_lane_seen_ms = loop_start
                if not lane_was_detected:
                    lane_was_detected = True
            elif lane_was_detected:
                lane_lost_count += 1
                lane_was_detected = False
            
            lane_missing_ms = time.ticks_diff(loop_start, last_lane_seen_ms)
            lane_confidence = lane_info.get("confidence", 0.0)
            lane_curve = lane_info.get("curve", 0)
            lane_width = lane_info.get("lane_width", 0)
            lane_samples = lane_info.get("sample_count", 0)
            lane_expected_samples = lane_info.get("expected_samples", 0)
            
            # Throttle is always passthrough (with smoothing)
            if throttle_in > 0:
                throttle_out = throttle_smoother.update(throttle_in)
                if throttle_out > 0:
                    pwm_throttle.pulse_width(throttle_out)
            else:
                throttle_out = 0
            
            # Steering logic
            if auto_mode:
                if not auto_was_enabled:
                    steering_pid.reset()
                auto_was_enabled = True
                try:
                    if lane_detected:
                        # Get steering PWM from PID controller
                        steering_pwm = steering_pwm_from_error(lane_error, img.width(), pid=steering_pid)
                        steering_out = steering_smoother.update(steering_pwm)
                        pwm_steering.pulse_width(steering_out)
                        last_auto_steering = steering_out
                        
                        if overlay_enabled:
                            img.draw_string((10, 10), f"AUTO - Error: {lane_error}px", color=(255, 255, 0))
                            img.draw_string((10, 25), f"Steer: {steering_out}us", color=(255, 255, 0))
                    elif lane_missing_ms < config_value(config, "LANE_HOLD_LAST_MS"):
                        steering_out = last_auto_steering
                        pwm_steering.pulse_width(steering_out)
                        if overlay_enabled:
                            img.draw_string((10, 10), "AUTO - HOLD LAST", color=(255, 128, 0))
                    else:
                        # No lanes detected - slow down and then center steering
                        steering_pid.reset()
                        steering_out = steering_smoother.update(1500)
                        pwm_steering.pulse_width(steering_out)
                        if overlay_enabled:
                            img.draw_string((10, 10), "AUTO - NO LANES!", color=(255, 0, 0))
                except Exception as e:
                    print(f"ERROR in auto mode: {e}")
                    # Failsafe - center steering
                    steering_out = 1500
                    pwm_steering.pulse_width(steering_out)
                    if overlay_enabled:
                        img.draw_string((10, 10), "AUTO - ERROR!", color=(255, 0, 0))
            else:
                if auto_was_enabled:
                    steering_pid.reset()
                auto_was_enabled = False
                # Manual mode - passthrough RC steering
                if steering_in > 0:
                    steering_out = steering_smoother.update(steering_in)
                    pwm_steering.pulse_width(steering_out)
                if overlay_enabled:
                    img.draw_string((10, 10), "MANUAL MODE", color=(0, 255, 0))
            
            if auto_mode:
                if lane_detected:
                    throttle_out = auto_throttle_limit(throttle_out, lane_error, lane_curve, lane_confidence, config)
                elif lane_missing_ms >= config_value(config, "LANE_HOLD_LAST_MS"):
                    throttle_out = min(throttle_out, THROTTLE_NEUTRAL)
                
                if lane_missing_ms >= config_value(config, "LANE_CENTER_FAILSAFE_MS"):
                    throttle_out = min(throttle_out, THROTTLE_NEUTRAL)
                
                if throttle_out > 0:
                    pwm_throttle.pulse_width(throttle_out)
            
            # Calculate loop time for debugging
            loop_time = time.ticks_diff(time.ticks_ms(), loop_start)
            if loop_time > 0:
                fps = 1000.0 / loop_time
                min_fps = min(min_fps, fps)
                max_fps = max(max_fps, fps)
            
            largest_blob_area = 0
            for blob in blobs:
                largest_blob_area = max(largest_blob_area, blob.area())
            
            mode_str = "auto" if auto_mode else "manual"
            lane_center_text = str(lane_center_x) if lane_detected else "none"
            status_text = (
                "OpenMV Rover Telemetry\n"
                "---------------------\n"
                f"Mode: {mode_str}  AutoSwitch:{mode_in:4d}  Threshold:{MODE_THRESHOLD}\n"
                f"Throttle In/Out: {throttle_in:4d} / {throttle_out:4d} us\n"
                f"Steering In/Out: {steering_in:4d} / {steering_out:4d} us\n"
                f"Lane Detected: {lane_detected}  CenterX:{lane_center_text}  Error:{lane_error:+7.1f} px  Heading:{heading_error:+6.1f} px\n"
                f"Confidence:{lane_confidence:.2f}  Curve:{lane_curve:+4d} px  Width:{lane_width:4d} px\n"
                f"Samples:{lane_samples}/{lane_expected_samples}  Lost Count:{lane_lost_count}  Missing:{lane_missing_ms} ms\n"
                f"Image CenterX: {image_center:4d}  Width:{img.width()}  Height:{img.height()}\n"
                f"Blobs: {len(blobs):2d}  Largest Area:{largest_blob_area:5d}\n"
                f"FPS: {fps:.1f}  Min:{min_fps:.1f}  Max:{max_fps:.1f}  Loop:{loop_time} ms  Frame:{frame_count}\n"
                f"AutoThrottle:{config_value(config, 'AUTO_THROTTLE_ENABLED')}  Max:{config_value(config, 'AUTO_THROTTLE_MAX')}  Neutral:{THROTTLE_NEUTRAL}\n"
                f"Lookahead:{lookahead_percent}%  CenterSmoothing:{config_value(config, 'LANE_CENTER_SMOOTHING'):.2f}\n"
                f"TopDown:{config_value(config, 'USE_TOP_DOWN_VIEW')}  ROI Top:{config_value(config, 'LANE_ROI_TOP_PERCENT')}%  Fill:{config_value(config, 'DRAW_LANE_FILL')}\n"
                f"USB Connected:{usb_connected}  Overlay Drawing:{overlay_enabled}\n"
                f"Config Saved:{config.get('_saved', False)}\n"
                "WiFi video: disabled; telemetry only"
            )
            
            if overlay_enabled:
                img.draw_string((10, 40), f"Thr: {throttle_in} Str: {steering_in}", color=(255, 255, 255))
                img.draw_string((10, 55), f"Mode: {mode_in}", color=(255, 255, 255))
            update_wifi_status(wifi_stream, status_text, config, usb_connected=usb_connected)
            
            if usb_connected and frame_count % SERIAL_PRINT_INTERVAL == 0:
                print(status_text.replace("\n", " | "))
    finally:
        ch1.deinit()
        ch2.deinit()
        ch3.deinit()
        pwm_throttle.deinit()
        pwm_steering.deinit()
        stop_wifi_stream(wifi_stream)

if __name__ == "__main__":
    main()
