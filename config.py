"""
config.py — All tunable constants and hardware port assignments.

Edit THIS FILE when tuning at the demo table. Everything that might
change between runs (PID gains, distances, thresholds) lives here.
"""

import math

# ═══════════════════════════════════════════════
#  HARDWARE PORT ASSIGNMENTS
# ═══════════════════════════════════════════════

# Drive motors
LEFT_MOTOR_PORT  = "A"
RIGHT_MOTOR_PORT = "D"

# Gripper motors (one per cube)
GRIPPER_1_PORT = "B"   # grabs cube 1
GRIPPER_2_PORT = "C"   # grabs cube 2
GRIPPER_1_PORT = "D"   
GRIPPER_2_PORT = "C"   

# Sensors
GYRO_PORT       = 2    
COLOR_PORT      = 1    
ULTRASONIC_PORT = 3    
TOUCH_ESTOP_PORT = 4   

# ═══════════════════════════════════════════════
#  PHYSICAL DIMENSIONS  (measure on YOUR robot)
# ═══════════════════════════════════════════════

WHEEL_DIAMETER_CM = 5.6        # outer diameter of the large EV3 wheel
WHEEL_CIRCUMFERENCE_CM = math.pi * WHEEL_DIAMETER_CM
DEGREES_PER_CM = 360.0 / WHEEL_CIRCUMFERENCE_CM   # encoder degrees per cm traveled
WHEELBASE_CM = 16.0            # center-to-center distance between the two drive wheels

# ═══════════════════════════════════════════════
#  DRIVE PID — straight line driving
# ═══════════════════════════════════════════════

DRIVE_BASE_SPEED  = 25    # percent power — keep LOW for reliability
DRIVE_KP          = 1.5   # proportional gain
DRIVE_KI          = 0.0   # integral gain — leave at 0 unless persistent drift
DRIVE_KD          = 0.8   # derivative gain

# ═══════════════════════════════════════════════
#  TURN PID — in-place gyro turns
# ═══════════════════════════════════════════════

TURN_KP           = 2.0
TURN_KD           = 0.5
TURN_TOLERANCE_DEG = 2.0   # stop turning when within this many degrees
TURN_MAX_POWER    = 40     # clamp turn power to prevent overshoot
TURN_MIN_POWER    = 8      # minimum power so the robot actually moves

# ═══════════════════════════════════════════════
#  WALL FOLLOWING PID
# ═══════════════════════════════════════════════

WALL_TARGET_DIST_CM = 15.0   # desired distance from wall
WALL_KP             = 1.2
WALL_KD             = 0.6

# ═══════════════════════════════════════════════
#  ULTRASONIC SENSOR FILTERING
# ═══════════════════════════════════════════════

US_MIN_RANGE_CM    = 3      # reject readings below this
US_MAX_RANGE_CM    = 50     # reject readings above this
US_MEDIAN_WINDOW   = 3      # median filter window size

# ═══════════════════════════════════════════════
#  GYRO SENSOR FILTERING
# ═══════════════════════════════════════════════

GYRO_MEDIAN_WINDOW     = 5     # median filter window size
GYRO_CALIBRATION_TIME  = 3.0   # seconds to hold still during calibration

# ═══════════════════════════════════════════════
#  COLOR SENSOR — THRESHOLDS
#
#  !! CALIBRATE THESE ON THE ACTUAL MAP !!
#
#  Run sensors.py standalone to print raw RGB values
#  and update these thresholds accordingly.
# ═══════════════════════════════════════════════

COLOR_MEAN_WINDOW = 3   # mean filter window size for RGB smoothing

# Thresholds are (R_min, R_max, G_min, G_max, B_min, B_max)
# These are PLACEHOLDER values — you MUST calibrate on the real map.
COLOR_THRESHOLDS = {
    "YELLOW":  (150, 255,  120, 255,   0,  80),
    "BLUE":    (  0,  80,    0, 100, 120, 255),
    "ORANGE":  (160, 255,   60, 140,   0,  60),
    "GREEN":   (  0, 100,  130, 255,   0, 100),
    "RED":     (140, 255,    0,  80,   0,  80),
    # WHITE is the fallback — anything that doesn't match above
}

# ═══════════════════════════════════════════════
#  GRIPPER CONSTANTS
# ═══════════════════════════════════════════════

GRIPPER_PINCH_POWER  =  30    # power to close the gripper
GRIPPER_LIFT_POWER   =  40    # power to lift after pinching
GRIPPER_HOLD_POWER   =  10    # small holding torque while driving
GRIPPER_RELEASE_POWER = -30   # reverse power to open and drop cube
GRIPPER_PINCH_TIME   = 0.5    # seconds to close
GRIPPER_LIFT_TIME    = 0.8    # seconds to lift
GRIPPER_RELEASE_TIME = 0.5    # seconds to open

# ═══════════════════════════════════════════════
#  PICKUP SEQUENCE — HARDCODED POSITIONS
#
#  Since the pharmacy counter is a virtual wall,
#  we hardcode the path from the start position
#  to each cube. Encoders are fresh at this point
#  so odometry is highly accurate.
#
#  !! MEASURE THESE ON THE ACTUAL MAP !!
# ═══════════════════════════════════════════════

CUBE_1_FORWARD_CM  = 10.0    # distance forward from start to cube 1
CUBE_1_HEADING_DEG = 0.0     # heading to face cube 1 (probably straight)
CUBE_2_LATERAL_CM  = 5.0     # lateral offset from cube 1 to cube 2
CUBE_2_HEADING_DEG = 0.0     # heading to face cube 2
PICKUP_EXIT_HEADING = 0.0    # heading to face the corridor after pickup

# ═══════════════════════════════════════════════
#  NAVIGATION — MAP KNOWLEDGE
# ═══════════════════════════════════════════════

MAP_SIZE_CM = 120.0           # the map is 120cm x 120cm
DOORWAY_WIDTH_CM = 24.0       # all doorways are 24cm wide

# ═══════════════════════════════════════════════
#  TIMING / SAFETY
# ═══════════════════════════════════════════════

CONTROL_LOOP_INTERVAL = 0.02   # 50 Hz control loop (seconds)
MISSION_TIMEOUT_SEC   = 170    # abort if mission exceeds ~2:50 (safety margin)
STATE_TIMEOUT_SEC     = 30     # abort a single state if stuck for 30s

# ═══════════════════════════════════════════════
#  SOUND — TODO: create your delivery and mission sounds
# ═══════════════════════════════════════════════

# These are placeholder parameters for the Sound class from utils/sound.py.
# Customize the pitch, duration, etc. to your liking.
DELIVERY_SOUND_PITCH    = "C5"
DELIVERY_SOUND_DURATION = 0.5
MISSION_SOUND_PITCH     = "G5"
MISSION_SOUND_DURATION  = 1.0
