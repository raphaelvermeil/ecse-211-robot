#!/usr/bin/env python3

import time
from utils.brick import BP
from config import *
from sensors import (
    init_sensors, is_estop_pressed, get_heading,
    classify_tile, classify_bed, reset_encoders,
    GRIPPER_1, GRIPPER_2
)
from navigation import (
    drive_straight, turn_to, stop_motors, stop_all,
    wall_follow_until_color, wall_follow_for_distance,
    drive_until_color, grab, release,
    play_delivery_sound, play_mission_sound,
    get_distance_traveled, sweep, turn_until_color
)



if __name__ == "__main__":
#     stop_all()


    #START INIT
    init_sensors()
    reset_encoders()
    mission_start_time = time.time()
    log("All systems go.")
    return State.PICKUP


def handle_pickup():
    """Pick up both foam cubes from the pharmacy counter.

    Since the counter is a virtual wall with no sensor feedback,
    we hardcode the displacement from the known starting position.
    Encoders are freshly reset so odometry is at its most accurate.
    """
    global packages_carried

    log("Starting pickup sequence...")

    # ── Drive to cube 1 position ──
    current_heading = get_heading()
      
    
    #GRAB BOTH
    grab(GRIPPER_1)
    packages_carried += 1
    log(f"Cube 1 grabbed. Carrying {packages_carried}.")

    # ── Reposition to cube 2 ──
    # TODO: The exact maneuver here depends on how the cubes are laid out
    #       on your pharmacy counter. You may need a combination of:
    #         turn_to(some_angle)
    #         drive_straight(heading, CUBE_2_LATERAL_CM)
    #       Measure on the actual map and adjust these calls.

    if abs(CUBE_2_HEADING_DEG - get_heading()) > TURN_TOLERANCE_DEG:
        if not turn_to(CUBE_2_HEADING_DEG):
            return State.DONE

    if CUBE_2_LATERAL_CM > 0:
        if not drive_straight(CUBE_2_HEADING_DEG, CUBE_2_LATERAL_CM):
            return State.DONE

    # Grab cube 2
    log("Grabbing cube 2...")
    grab(GRIPPER_2)
    
    
    #GETTING OUT OF BLUE AND INTO 1ST ROOM
    drive_straight(current_heading, -15)
    turn_to(-90, "ccw")
    current_heading = get_heading()
    drive_until_color(current_heading, ["YELLOW"], "front")
    current_heading = get_heading()
    
    
    #SWEEP FIRST ROOM
    result, room_depth = sweep(current_heading)
    print(result)
    print(room_depth)
    if result == "GREEN":
        release(GRIPPER_1)


    #GETTING OUT OF FIRST ROOM AND INTO SECOND ROOM
    turn_to(90, "cw")
    drive_straight(90, room_depth)
    turn_to(0, "ccw")
    drive_straight(0, 47)
    turn_to(-90, "ccw")
    drive_until_color(-90, "YELLOW", "FRONT")
    
    
    #SWEEP SECOND ROOM
    result, room_depth = sweep(current_heading)
    print(result)
    print(room_depth)
    if result == "GREEN":
        release(GRIPPER_2)
        
        
    #GETTING OUT OF 2ND ROOM AND INTO 3RD ROOM
    turn_to(90 , "cw")
    drive_straight(90, room_depth)
    turn_to(0, "ccw")
    drive_straight(0, 25)
    turn_to(90, "cw")
    drive_until_color(90, "YELLOW", "FRONT")
    
    
    #SWEEP PART 1 OF 3RD ROOM
    result, room_depth = sweep(current_heading, 25, 15, 20)
    
    
    #GET TO PART 2 OF 3RD ROOM
    turn_to(-90, "ccw")
    drive_straight(-90, room_depth)
    turn_to(0, "cw")
    drive_straight(0, 10)
    turn_to(90, "cw")
    
    
    #SWEEP PART 2 OF 3RD ROOM
    result, room_depth = sweep(current_heading, 25, 15, 20)
    
    
    #GETTING OUT OF 3RD ROOM AND INTO BLUE ZONE
    turn_to(270, "cw")
    drive_straight(270, room_depth)
    turn_to(180, "ccw") # MAYBE NEGATIVE?
    drive_straight(180, 40)
    turn_to(90, "ccw")
    drive_until_color(90, "BLUE", "FRONT")
    drive_straight(90, 10)
    stop_all()

    #THE END!!


