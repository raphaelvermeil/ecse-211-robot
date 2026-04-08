def turn_to(target_heading, direction=None):

    prev_error = 0.0
    stall_count = 0
 
    while True:
        if is_estop_pressed():
            stop_motors()
            return False
 
        current = get_heading()
        print(current)
        error = target_heading - current
 
        # Normalize error based on desired direction
        if direction == "cw":
            # Clockwise = negative error (we want error < 0)
            while error > 0:
                error -= 360
            # If we're already past target (error ≈ -360), wrap to near 0
            while error < -360:
                error += 360
        elif direction == "ccw":
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