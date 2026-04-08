

import time
import math
from utils.brick import BP, Motor
from utils.sound import Sound
from config import *
from sensors import (
    LEFT_MOTOR, RIGHT_MOTOR, GRIPPER_1, GRIPPER_2,
    get_heading, get_wall_distance, get_left_encoder, get_right_encoder,
    reset_encoders, is_estop_pressed, classify_tile, classify_bed,
    get_color_rgb
)



_delivery_sound = Sound(
    duration=DELIVERY_SOUND_DURATION,
    pitch=DELIVERY_SOUND_PITCH,
    volume=60
)

_mission_sound = Sound(
    duration=MISSION_SOUND_DURATION,
    pitch=MISSION_SOUND_PITCH,
    volume=60
)





def _encoder_to_cm(degrees):
    """Convert encoder degrees to centimeters traveled."""
    return degrees / DEGREES_PER_CM


def get_distance_traveled():
    """Return distance traveled in cm since last encoder reset.
    Uses the average of both wheels."""
    left_cm = _encoder_to_cm(get_left_encoder())
    right_cm = _encoder_to_cm(get_right_encoder())
    return (left_cm + right_cm) / 2.0


# ═══════════════════════════════════════════════
#  MOTOR CONTROL
# ═══════════════════════════════════════════════

def stop_motors():
    """Immediately stop both drive motors."""
    LEFT_MOTOR.set_power(0)
    RIGHT_MOTOR.set_power(0)


def stop_all():
    """Stop everything — drive motors and grippers."""
    BP.reset_all()


# ═══════════════════════════════════════════════
#  PID STRAIGHT DRIVING
# ═══════════════════════════════════════════════

def drive_straight(heading, distance_cm):
    """Drive straight for a given distance while maintaining heading.

    Uses gyro-PID to correct for motor imbalance and drift.
    Returns True if completed, False if interrupted by e-stop.

    Args:
        heading: target heading in degrees (from gyro)
        distance_cm: distance to travel in cm
    """
    reset_encoders()
    prev_error = 0.0
    integral = 0.0

    while abs(get_distance_traveled()) < abs(distance_cm):
        # E-stop check
        if is_estop_pressed():
            
            
            stop_motors()
            
            return False

        # PID correction
        current = get_heading()
        error = heading - current
        integral += error
        derivative = error - prev_error

        correction = (DRIVE_KP * error +
                      DRIVE_KI * integral +
                      DRIVE_KD * derivative)

        # Determine direction
        if distance_cm >= 0:
            left_power  = DRIVE_BASE_SPEED + correction
            right_power = DRIVE_BASE_SPEED - correction
        else:
            # Driving backward
            left_power  = -DRIVE_BASE_SPEED + correction
            right_power = -DRIVE_BASE_SPEED - correction

        # Clamp power to safe range
        left_power  = max(min(left_power, 60), -60)
        right_power = max(min(right_power, 60), -60)

        LEFT_MOTOR.set_power(left_power)
        RIGHT_MOTOR.set_power(right_power)

        prev_error = error
        time.sleep(CONTROL_LOOP_INTERVAL)

    stop_motors()
    return True


# ═══════════════════════════════════════════════
#  PID TURNING
# ═══════════════════════════════════════════════

def turn_to(target_heading, direction=None):

    prev_error = 0.0
    stall_count = 0
 
    while True:
        if is_estop_pressed():
            stop_motors()
            return False
 
        current = get_heading()
        error = target_heading - current
 
        # Normalize error based on desired direction
        if direction == "ccw":
            # Clockwise = negative error (we want error < 0)
            while error > 0:
                error -= 360
            # If we're already past target (error ≈ -360), wrap to near 0
            while error < -360:
                error += 360
        elif direction == "cw":
            # Counter-clockwise = positive error (we want error > 0)
            while error < 0:
                error += 360
            while error > 360:
                error -= 360
        else:
            # Default: shortest path (-180..180)
            while error > 180:
                error -= 360
            while error < -180:
                error += 360
 
        # Done?
        if abs(error) < TURN_TOLERANCE_DEG:
            break
 
        derivative = error - prev_error
        power = (TURN_KP * error + TURN_KD * derivative)/2
 
        # Clamp
        power = max(min(power, TURN_MAX_POWER), -TURN_MAX_POWER)
 
        # Ensure minimum power so the robot actually moves
        if 0 < power < TURN_MIN_POWER:
            power = TURN_MIN_POWER
        elif -TURN_MIN_POWER < power < 0:
            power = -TURN_MIN_POWER
 
        # Spin in place: left motor backward, right motor forward → clockwise
        LEFT_MOTOR.set_power(-power)
        RIGHT_MOTOR.set_power(power)
 
        prev_error = error
        time.sleep(CONTROL_LOOP_INTERVAL)
 
        # Safety: detect if we're stuck
        stall_count += 1
        if stall_count > 500:  # ~10 seconds at 50Hz
            print("WARNING: turn_to stalled, aborting")
            break
 
    stop_motors()
    time.sleep(0.1)  # brief settle time
    return True




# def turn_to(target_heading):
#     """Turn in place to face the target heading using gyro-PID.
# 
#     Returns True if completed, False if interrupted by e-stop.
# 
#     Args:
#         target_heading: desired heading in degrees
#     """
#     prev_error = 0.0
#     stall_count = 0
# 
#     while True:
#         if is_estop_pressed():
#             stop_motors()
#             return False
# 
#         current = get_heading()
#         print(current)
#         error = target_heading - current
# 
#         # Normalize error to -180..180
#         while error > 180:
#             error -= 360
#         while error < -180:
#             error += 360
# 
#         # Done?
#         if abs(error) < TURN_TOLERANCE_DEG:
#             break
# 
#         derivative = error - prev_error
#         power = (TURN_KP * error + TURN_KD * derivative)/2
# 
#         # Clamp
#         power = max(min(power, TURN_MAX_POWER), -TURN_MAX_POWER)
# 
#         # Ensure minimum power so the robot actually moves
#         if 0 < power < TURN_MIN_POWER:
#             power = TURN_MIN_POWER
#         elif -TURN_MIN_POWER < power < 0:
#             power = -TURN_MIN_POWER
# 
#         # Spin in place: left motor backward, right motor forward → clockwise
#         LEFT_MOTOR.set_power(-power)
#         RIGHT_MOTOR.set_power(power)
# 
#         prev_error = error
#         time.sleep(CONTROL_LOOP_INTERVAL)
# 
#         # Safety: detect if we're stuck
#         stall_count += 1
#         if stall_count > 500:  # ~10 seconds at 50Hz
#             print("WARNING: turn_to stalled, aborting")
#             break
# 
#     stop_motors()
#     time.sleep(0.1)  # brief settle time
#     return True




def turn_until_color(target_heading, target_color):
    
    prev_error = 0.0
    stall_count = 0

    while True:
        if is_estop_pressed():
            stop_motors()
            return "ESTOP"

        # Check for target color
        tile = classify_tile()
        if tile == target_color:
            stop_motors()
            return "COLOR_DETECTED"

        current = get_heading()
        error = target_heading - current

        # Normalize error to -180..180
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        # Done?
        if abs(error) < TURN_TOLERANCE_DEG:
            stop_motors()
            time.sleep(0.1)
            return "REACHED_HEADING"

        derivative = error - prev_error
        power = TURN_KP * error + TURN_KD * derivative

        # Clamp
        power = max(min(power, TURN_MAX_POWER), -TURN_MAX_POWER)

        # Ensure minimum power so the robot actually moves
        if 0 < power < TURN_MIN_POWER:
            power = TURN_MIN_POWER
        elif -TURN_MIN_POWER < power < 0:
            power = -TURN_MIN_POWER

        LEFT_MOTOR.set_power(-power)
        RIGHT_MOTOR.set_power(power)

        prev_error = error
        time.sleep(CONTROL_LOOP_INTERVAL)

        stall_count += 1
        if stall_count > 500:
            print("WARNING: turn_to_or_color stalled, aborting")
            stop_motors()
            return "STALLED"

# ═══════════════════════════════════════════════
#  WALL FOLLOWING
# ═══════════════════════════════════════════════

def wall_follow_step(prev_error):
    """Execute one iteration of wall-following PID.

    Call this in a loop while navigating corridors.
    Returns the current error for the next iteration.

    Args:
        prev_error: error value from the previous iteration
    Returns:
        current error (pass back to next call)
    """
    dist = get_wall_distance()
    error = WALL_TARGET_DIST_CM - dist
    derivative = error - prev_error

    correction = WALL_KP * error + WALL_KD * derivative

    # Correction sign depends on which side the wall is on.
    # Assuming ultrasonic points RIGHT:
    #   too close to wall (error > 0) → steer left → slow right, speed left
    #   too far from wall (error < 0) → steer right → slow left, speed right
    LEFT_MOTOR.set_power(DRIVE_BASE_SPEED + correction)
    RIGHT_MOTOR.set_power(DRIVE_BASE_SPEED - correction)

    return error


def wall_follow_for_distance(distance_cm):
    """Wall-follow for a given distance.

    Returns True if completed, False if e-stop or tile detected.
    """
    reset_encoders()
    prev_error = 0.0

    while abs(get_distance_traveled()) < distance_cm:
        if is_estop_pressed():
            stop_motors()
            return False
        prev_error = wall_follow_step(prev_error)
        time.sleep(CONTROL_LOOP_INTERVAL)

    stop_motors()
    return True


def wall_follow_until_color(target_colors):
    """Wall-follow until the color sensor detects one of the target colors.

    Args:
        target_colors: list of color strings, e.g. ["YELLOW", "ORANGE"]
    Returns:
        The detected color string, or None if e-stop pressed.
    """
    prev_error = 0.0

    while True:
        if is_estop_pressed():
            stop_motors()
            return None

        # Check for target color
        tile = classify_tile()
        if tile in target_colors:
            stop_motors()
            return tile

        prev_error = wall_follow_step(prev_error)
        time.sleep(CONTROL_LOOP_INTERVAL)


# ═══════════════════════════════════════════════
#  DRIVE UNTIL COLOR
# ═══════════════════════════════════════════════

def drive_until_color(heading, target_colors, direction, max_distance_cm=100):
    """Drive straight until a target color is detected.

    Args:
        heading: heading to maintain
        target_colors: list of color strings to look for
        max_distance_cm: safety limit
    Returns:
        The detected color string, or None.
    """
    reset_encoders()
    prev_error = 0.0
    integral = 0.0
    

    while abs(get_distance_traveled()) < max_distance_cm:
        if is_estop_pressed():
            
            stop_motors()
            
            return None

        tile = classify_tile()
        if tile in target_colors:
            stop_motors()
            return tile

        # PID straight
        current = get_heading()
        error = heading - current
        integral += error
        derivative = error - prev_error
        correction = DRIVE_KP * error + DRIVE_KI * integral + DRIVE_KD * derivative

        if direction == "BACK":
            LEFT_MOTOR.set_power(-(DRIVE_BASE_SPEED + correction))
            RIGHT_MOTOR.set_power(-(DRIVE_BASE_SPEED - correction))
        else:
            LEFT_MOTOR.set_power(DRIVE_BASE_SPEED - correction)
            RIGHT_MOTOR.set_power(DRIVE_BASE_SPEED + correction)

        prev_error = error
        time.sleep(CONTROL_LOOP_INTERVAL)

    stop_motors()
    return None


# ═══════════════════════════════════════════════
#  GRIPPER CONTROL
# ═══════════════════════════════════════════════

def grab(gripper_motor):
    """Close the gripper to pinch a cube, then lift.
    Keeps a small holding torque after lifting.

    Args:
        gripper_motor: Motor object (GRIPPER_1 or GRIPPER_2)
    """
    gripper_motor.set_power(GRIPPER_PINCH_POWER)
    time.sleep(GRIPPER_PINCH_TIME)
    gripper_motor.set_power(GRIPPER_LIFT_POWER)
    time.sleep(GRIPPER_LIFT_TIME)
    gripper_motor.set_power(GRIPPER_HOLD_POWER)  # hold torque


def release(gripper_motor):
    """Open the gripper to release a cube.

    Args:
        gripper_motor: Motor object (GRIPPER_1 or GRIPPER_2)
    """
    gripper_motor.set_power(GRIPPER_RELEASE_POWER)
    time.sleep(GRIPPER_RELEASE_TIME)
    gripper_motor.set_power(0)


# ═══════════════════════════════════════════════
#  SOUNDS
# ═══════════════════════════════════════════════

def play_delivery_sound():
    """Play a short sound after a successful delivery."""
    try:
        _delivery_sound.play()
        _delivery_sound.wait_done()
    except Exception as e:
        print(f"Sound error: {e}")


def play_mission_sound():
    """Play a sound when the full mission is complete."""
    try:
        _mission_sound.play()
        _mission_sound.wait_done()
    except Exception as e:
        print(f"Sound error: {e}")
        
        
        
        
def sweep(base_heading, sweep_angle=25, forward_speed=15, max_distance_cm=50):
    
    
    reset_encoders()
    
    # Sweep targets: right, left, right, left, ...
    targets = [
        base_heading + sweep_angle,
        base_heading - sweep_angle,
    ]
    sweep_index = 0
    current_target = targets[0]
    
    prev_error = 0.0
    
    while abs(get_distance_traveled()) < max_distance_cm:
        if is_estop_pressed():
            
            stop_motors()
            return "UNKNOWN"
        
        # Check for bed sticker
        bed = classify_bed()
        if bed in ("GREEN", "RED"):
            stop_motors()
            distance_traveled = abs(get_distance_traveled())
            return bed, distance_traveled
        
        # PID toward current sweep target
        current = get_heading()
        error = current_target - current
        
        # Normalize to -180..180
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        derivative = error - prev_error
        turn_correction = TURN_KP * error + TURN_KD * derivative
        turn_correction = max(min(turn_correction, 25), -25)
        
        # Combine: slow forward + turning correction
        LEFT_MOTOR.set_power(-(forward_speed + turn_correction))
        RIGHT_MOTOR.set_power(-(forward_speed - turn_correction))
        
        # Switch sweep direction when close to target
        if abs(error) < 5:
            sweep_index = (sweep_index + 1) % len(targets)
            current_target = targets[sweep_index]
        
        prev_error = error
        time.sleep(CONTROL_LOOP_INTERVAL)
    
    # Reached max distance without finding a bed
    stop_motors()
    return "UNKNOWN", abs(get_distance_traveled())


if __name__ == "__main__":
    




    init_sensors()

    reset_encoders()
    current_heading = get_heading()
    turn_to(90, "cw")
    turn_to(270, "cw")
    
    stop_all()
    