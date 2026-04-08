def sweep_for_bed(base_heading, sweep_angle=30, forward_speed=15, max_distance_cm=40):
    """Sweep left-right while advancing to find a bed sticker.
    
    The robot oscillates its heading by ±sweep_angle degrees around
    base_heading while creeping forward. The color sensor polls
    every iteration looking for GREEN or RED.
    
    Args:
        base_heading: the heading the robot entered the room at
        sweep_angle: degrees to sweep each direction (default 30)
        forward_speed: slow forward power during sweep
        max_distance_cm: stop sweeping after this distance
    Returns:
        "GREEN", "RED", or "UNKNOWN" if nothing found
    """
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
            return bed
        
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
        LEFT_MOTOR.set_power(forward_speed - turn_correction)
        RIGHT_MOTOR.set_power(forward_speed + turn_correction)
        
        # Switch sweep direction when close to target
        if abs(error) < 5:
            sweep_index = (sweep_index + 1) % len(targets)
            current_target = targets[sweep_index]
        
        prev_error = error
        time.sleep(CONTROL_LOOP_INTERVAL)
    
    # Reached max distance without finding a bed
    stop_motors()
    return "UNKNOWN"