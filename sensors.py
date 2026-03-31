"""
sensors.py — Sensor initialization, filtering, and clean reading functions.

This module wraps the raw BrickPi sensor APIs with filters from utils/filters.py.
It exports simple functions that the navigation layer calls:

    get_heading()       → filtered gyro heading in degrees
    get_wall_distance() → filtered ultrasonic distance in cm
    get_color_rgb()     → filtered (R, G, B) tuple
    classify_tile()     → string: "YELLOW", "BLUE", "ORANGE", "WHITE", etc.
    classify_bed()      → string: "GREEN", "RED", or "UNKNOWN"
    is_estop_pressed()  → bool
    get_left_encoder()  → raw encoder degrees (left motor)
    get_right_encoder() → raw encoder degrees (right motor)

Run this file standalone (python3 sensors.py) for a calibration tool
that prints raw sensor values to help you set config.py thresholds.
"""

import time
from utils.brick import (
    BP, Motor, TouchSensor, EV3UltrasonicSensor,
    EV3ColorSensor, EV3GyroSensor, wait_ready_sensors, SensorError
)
from utils.filters import MedianWindow, MeanWindow
from config import *


# ═══════════════════════════════════════════════
#  HARDWARE INITIALIZATION
# ═══════════════════════════════════════════════

# Drive motors (used here only for encoder reads — navigation.py controls them)
LEFT_MOTOR  = Motor(LEFT_MOTOR_PORT)
RIGHT_MOTOR = Motor(RIGHT_MOTOR_PORT)

# Gripper motors
GRIPPER_1 = Motor(GRIPPER_1_PORT)
GRIPPER_2 = Motor(GRIPPER_2_PORT)

# Sensors
GYRO       = EV3GyroSensor(GYRO_PORT, mode="both")
COLOR      = EV3ColorSensor(COLOR_PORT, mode="component")
ULTRASONIC = EV3UltrasonicSensor(ULTRASONIC_PORT, mode="cm")
TOUCH_ESTOP = TouchSensor(TOUCH_ESTOP_PORT)


# ═══════════════════════════════════════════════
#  FILTERS
# ═══════════════════════════════════════════════

_gyro_filter  = MedianWindow(GYRO_MEDIAN_WINDOW)
_us_filter    = MedianWindow(US_MEDIAN_WINDOW)
_color_r_filter = MeanWindow(COLOR_MEAN_WINDOW)
_color_g_filter = MeanWindow(COLOR_MEAN_WINDOW)
_color_b_filter = MeanWindow(COLOR_MEAN_WINDOW)


# ═══════════════════════════════════════════════
#  GYRO CALIBRATION STATE
# ═══════════════════════════════════════════════

_gyro_offset = 0.0       # initial heading offset
_gyro_drift_rate = 0.0   # degrees per second of drift
_gyro_start_time = 0.0   # when calibration finished


# ═══════════════════════════════════════════════
#  INITIALIZATION
# ═══════════════════════════════════════════════

def init_sensors():
    """Call once at program start. Waits for all sensors to be ready,
    then calibrates the gyro. KEEP THE ROBOT PERFECTLY STILL."""
    print("Initializing sensors, keep the robot still...")
    wait_ready_sensors()
    calibrate_gyro()
    print("Sensors ready!")


def calibrate_gyro():
    """Measure gyro drift while the robot is stationary.
    Must be called while the robot is NOT moving."""
    global _gyro_offset, _gyro_drift_rate, _gyro_start_time

    # Read initial value
    time.sleep(0.5)  # let sensor settle
    try:
        initial = GYRO.get_abs_measure()
        if initial is None:
            initial = 0
    except SensorError:
        initial = 0

    # Wait and measure drift
    time.sleep(GYRO_CALIBRATION_TIME)

    try:
        final = GYRO.get_abs_measure()
        if final is None:
            final = initial
    except SensorError:
        final = initial

    _gyro_drift_rate = (final - initial) / GYRO_CALIBRATION_TIME
    _gyro_offset = final  # so heading starts at 0
    _gyro_start_time = time.time()

    print(f"Gyro calibrated: offset={_gyro_offset:.1f}, drift={_gyro_drift_rate:.3f} deg/s")


# ═══════════════════════════════════════════════
#  READING FUNCTIONS
# ═══════════════════════════════════════════════

def get_heading():
    """Return the filtered, drift-compensated heading in degrees.
    Positive = clockwise from start orientation."""
    try:
        raw = GYRO.get_abs_measure()
        if raw is None:
            return _gyro_filter.get_value() or 0.0
    except SensorError:
        return _gyro_filter.get_value() or 0.0

    # Compensate for drift
    elapsed = time.time() - _gyro_start_time
    compensated = raw - _gyro_offset - (_gyro_drift_rate * elapsed)

    _gyro_filter.append(compensated)
    return _gyro_filter.get_value()


def get_wall_distance():
    """Return the filtered ultrasonic distance in cm.
    Returns the last good value if the current reading is out of range."""
    try:
        raw = ULTRASONIC.get_cm()
        if raw is None:
            return _us_filter.get_value() or 999.0
    except SensorError:
        return _us_filter.get_value() or 999.0

    # Reject implausible readings
    if raw < US_MIN_RANGE_CM or raw > US_MAX_RANGE_CM:
        last = _us_filter.get_value()
        return last if last is not None else 999.0

    _us_filter.append(raw)
    return _us_filter.get_value()


def get_color_rgb():
    """Return filtered (R, G, B) tuple from the color sensor."""
    try:
        val = COLOR.get_value()
        if val is None or len(val) < 3:
            r = _color_r_filter.get_value() or 0
            g = _color_g_filter.get_value() or 0
            b = _color_b_filter.get_value() or 0
            return (r, g, b)
    except SensorError:
        r = _color_r_filter.get_value() or 0
        g = _color_g_filter.get_value() or 0
        b = _color_b_filter.get_value() or 0
        return (r, g, b)

    _color_r_filter.append(val[0])
    _color_g_filter.append(val[1])
    _color_b_filter.append(val[2])

    return (
        _color_r_filter.get_value(),
        _color_g_filter.get_value(),
        _color_b_filter.get_value()
    )


def classify_tile():
    """Classify the floor tile based on color sensor RGB.
    Returns: "YELLOW", "BLUE", "ORANGE", "WHITE"."""
    r, g, b = get_color_rgb()
    return _classify_rgb(r, g, b, targets=["YELLOW", "BLUE", "ORANGE"])


def classify_bed():
    """Classify a bed sticker based on color sensor RGB.
    Returns: "GREEN", "RED", or "UNKNOWN"."""
    r, g, b = get_color_rgb()
    return _classify_rgb(r, g, b, targets=["GREEN", "RED"])


def _classify_rgb(r, g, b, targets):
    """Check RGB against thresholds in config for the given target colors.
    Returns the first match, or "WHITE"/"UNKNOWN" as fallback."""
    for color_name in targets:
        if color_name not in COLOR_THRESHOLDS:
            continue
        rmin, rmax, gmin, gmax, bmin, bmax = COLOR_THRESHOLDS[color_name]
        if rmin <= r <= rmax and gmin <= g <= gmax and bmin <= b <= bmax:
            return color_name
    return "WHITE" if "YELLOW" in targets else "UNKNOWN"


def is_estop_pressed():
    """Return True if the emergency stop touch sensor is pressed."""
    try:
        return TOUCH_ESTOP.is_pressed()
    except SensorError:
        return False


def get_left_encoder():
    """Return the left motor encoder in degrees."""
    try:
        return LEFT_MOTOR.get_encoder()
    except Exception:
        return 0


def get_right_encoder():
    """Return the right motor encoder in degrees."""
    try:
        return RIGHT_MOTOR.get_encoder()
    except Exception:
        return 0


def reset_encoders():
    """Reset both drive motor encoders to 0."""
    LEFT_MOTOR.reset_encoder()
    RIGHT_MOTOR.reset_encoder()


# ═══════════════════════════════════════════════
#  CALIBRATION TOOL — run standalone
# ═══════════════════════════════════════════════

if __name__ == "__main__":
    """
    Run this file directly to print live sensor values.
    Use this to calibrate your color thresholds in config.py.

    Place the robot on each surface and note the RGB values,
    then update COLOR_THRESHOLDS in config.py.
    """
    print("=== SENSOR CALIBRATION TOOL ===")
    print("Press Ctrl+C to exit.\n")

    init_sensors()

    try:
        while True:
            heading = get_heading()
            wall_dist = get_wall_distance()
            r, g, b = get_color_rgb()
            tile = classify_tile()
            left_enc = get_left_encoder()
            right_enc = get_right_encoder()
            estop = is_estop_pressed()

            print(
                f"Gyro: {heading:7.1f}° | "
                f"US: {wall_dist:5.1f}cm | "
                f"RGB: ({r:3.0f},{g:3.0f},{b:3.0f}) → {tile:8s} | "
                f"Enc L:{left_enc:6d} R:{right_enc:6d} | "
                f"E-Stop: {estop}",
                end="\r"
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nCalibration tool stopped.")
        BP.reset_all()
