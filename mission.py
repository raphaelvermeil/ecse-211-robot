#!/usr/bin/env python3
"""
mission.py — Main entry point for the Smart Hospital Assistant Robot.

This is the file you run: python3 mission.py

It contains the top-level state machine that orchestrates the full mission:
    STARTUP → PICKUP → NAVIGATE → ENTER_ROOM → SCAN_ROOM → DELIVER → RETURN → DONE

All sensor reads go through sensors.py.
All movement commands go through navigation.py.
All constants live in config.py.
"""

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
    get_distance_traveled
)


# ═══════════════════════════════════════════════
#  STATES
# ═══════════════════════════════════════════════

class State:
    STARTUP    = "STARTUP"
    PICKUP     = "PICKUP"
    NAVIGATE   = "NAVIGATE"
    ENTER_ROOM = "ENTER_ROOM"
    SCAN_ROOM  = "SCAN_ROOM"
    DELIVER    = "DELIVER"
    RETURN     = "RETURN"
    DONE       = "DONE"


# ═══════════════════════════════════════════════
#  MISSION STATE
# ═══════════════════════════════════════════════

state = State.STARTUP
packages_carried = 0
deliveries_completed = 0
rooms_visited = 0
mission_start_time = 0

# Track which gripper to release next.
# Gripper 1 grabbed first, so we deliver from gripper 1 first.
_gripper_queue = [GRIPPER_1, GRIPPER_2]


# ═══════════════════════════════════════════════
#  LOGGING
# ═══════════════════════════════════════════════

def log(msg):
    """Print a timestamped log message."""
    elapsed = time.time() - mission_start_time if mission_start_time else 0
    print(f"[{elapsed:6.1f}s] [{state:12s}] {msg}")


# ═══════════════════════════════════════════════
#  STATE HANDLERS
#
#  Each function handles one state and returns
#  the next state to transition to.
# ═══════════════════════════════════════════════

def handle_startup():
    """Initialize sensors, calibrate gyro, reset encoders."""
    global mission_start_time

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
    target = CUBE_1_HEADING_DEG

    if abs(target - current_heading) > TURN_TOLERANCE_DEG:
        if not turn_to(target):
            return State.DONE  # e-stop

    if not drive_straight(target, CUBE_1_FORWARD_CM):
        return State.DONE  # e-stop

    # Grab cube 1
    log("Grabbing cube 1...")
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
    packages_carried += 1
    log(f"Cube 2 grabbed. Carrying {packages_carried}.")

    # ── Face the corridor ──
    if not turn_to(PICKUP_EXIT_HEADING):
        return State.DONE

    log("Pickup complete. Heading to corridor.")
    return State.NAVIGATE


def handle_navigate():
    """Navigate corridors using wall following.
    Look for doorways (ORANGE tiles) or the pharmacy (BLUE tile).

    TODO: You will need to add map-specific logic here.
    For example, knowing which direction to turn after exiting a room,
    or which corridor to follow to reach the next room.
    The odometry tracker (heading + encoder distance) helps you know
    roughly where you are on the 120cm x 120cm map.
    """
    log("Navigating corridor, looking for doorways...")

    # Wall-follow until we see an orange (doorway) or blue (pharmacy) tile
    detected = wall_follow_until_color(["ORANGE", "YELLOW"])

    if detected is None:
        # E-stop was pressed
        return State.DONE

    if detected == "ORANGE" or detected == "YELLOW":
        log(f"Detected {detected} tile — doorway/room found!")
        return State.ENTER_ROOM

    # Fallback (shouldn't reach here)
    return State.NAVIGATE


def handle_enter_room():
    """Turn into the doorway and drive through it into the room.

    TODO: The turn direction (left or right) depends on which side
    of the corridor the room is on. You'll need map knowledge or
    an additional sensor check to decide.
    """
    global rooms_visited

    log("Entering room through doorway...")

    # TODO: Determine turn direction based on map knowledge / room index.
    #       For now, assume a 90° right turn. Adjust per your map layout.
    current_heading = get_heading()
    room_heading = current_heading + 90  # TODO: +90 or -90 depending on room side

    if not turn_to(room_heading):
        return State.DONE

    # Drive through the 24cm doorway + a bit more to get fully inside
    if not drive_straight(room_heading, DOORWAY_WIDTH_CM + 5):
        return State.DONE

    rooms_visited += 1
    log(f"Entered room #{rooms_visited}.")
    return State.SCAN_ROOM


def handle_scan_room():
    """Scan bed stickers to determine if delivery is needed.

    The robot must approach each bed indicator and read its color:
      GREEN → needs medication → transition to DELIVER
      RED   → no medication needed → skip

    TODO: The movement within the room to reach each bed sticker
    depends on the room layout. Single rooms have 1 bed, the
    double-occupancy room has 2 beds to check independently.

    You'll need to:
    1. Drive toward the bed location (hardcoded or wall-following)
    2. Position the color sensor over the sticker
    3. Read and classify
    4. Repeat for second bed in double rooms
    """
    log("Scanning bed stickers...")

    # TODO: Drive toward the first bed sticker position.
    #       This depends on your room layout. Example:
    #         drive_straight(get_heading(), 10)  # approach bed
    #
    #       For now we just read the color sensor where we are.

    # Read bed sticker
    bed_color = classify_bed()
    log(f"Bed sticker detected: {bed_color}")

    if bed_color == "GREEN" and packages_carried > 0:
        return State.DELIVER
    elif bed_color == "RED":
        log("Red bed — no delivery needed.")
        # TODO: Check if there's a second bed in this room (double occupancy).
        #       If yes, navigate to the second bed and scan again.
        #       If no more beds, exit the room and return to NAVIGATE.
        return _exit_room_and_navigate()
    else:
        log("Could not classify bed. Moving on.")
        # TODO: Retry scan or skip. For safety, skip and continue.
        return _exit_room_and_navigate()


def handle_deliver():
    """Deliver one foam cube to the green bed, play sound."""
    global packages_carried, deliveries_completed

    log("Delivering package...")

    # Release from the next gripper in queue
    if _gripper_queue:
        gripper = _gripper_queue.pop(0)
        release(gripper)
    else:
        log("ERROR: No grippers left to release!")

    packages_carried -= 1
    deliveries_completed += 1

    # Play delivery confirmation sound
    play_delivery_sound()
    log(f"Delivery #{deliveries_completed} complete. Carrying {packages_carried} remaining.")

    # Check if we're done with all deliveries
    if deliveries_completed >= 2:
        log("All deliveries complete! Returning to pharmacy.")
        return _exit_room_then(State.RETURN)
    else:
        # TODO: Check for a second bed in the same room before leaving.
        #       If there's another bed, scan it. Otherwise, exit and find the next room.
        return _exit_room_and_navigate()


def handle_return():
    """Navigate back to the pharmacy (blue tile) and end the mission.

    TODO: You'll need map-specific logic to navigate back.
    Options:
      1. Reverse the route using odometry
      2. Wall-follow back and look for the BLUE tile
      3. Hardcode a return path from each room
    """
    log("Returning to pharmacy...")

    # Wall-follow until we detect the blue pharmacy tile
    detected = wall_follow_until_color(["BLUE"])

    if detected is None:
        return State.DONE  # e-stop

    if detected == "BLUE":
        log("Pharmacy reached!")
        stop_motors()
        play_mission_sound()
        log("MISSION COMPLETE!")
        return State.DONE

    # If we didn't find blue, keep navigating
    # TODO: add a fallback strategy (e.g., turn around and try again)
    return State.RETURN


# ═══════════════════════════════════════════════
#  HELPER: EXIT A ROOM
# ═══════════════════════════════════════════════

def _exit_room_and_navigate():
    """Exit the current room through the doorway and resume corridor navigation."""
    return _exit_room_then(State.NAVIGATE)


def _exit_room_then(next_state):
    """Turn around, drive out through the doorway, resume given state.

    TODO: The exit maneuver depends on the room geometry.
    You may need to:
      1. Turn 180° to face the doorway
      2. Drive through the doorway
      3. Turn to face the corridor direction
    Adjust based on your map layout.
    """
    log("Exiting room...")

    # Turn 180° (or whatever angle faces the doorway from current position)
    current = get_heading()
    exit_heading = current + 180  # TODO: adjust based on room geometry
    turn_to(exit_heading)

    # Drive out through doorway
    drive_straight(exit_heading, DOORWAY_WIDTH_CM + 10)

    # TODO: Turn to face the corridor for the next navigation phase
    #       This depends on which direction the next room or pharmacy is.

    return next_state


# ═══════════════════════════════════════════════
#  MAIN LOOP
# ═══════════════════════════════════════════════

def main():
    global state

    # Map states to their handler functions
    handlers = {
        State.STARTUP:    handle_startup,
        State.PICKUP:     handle_pickup,
        State.NAVIGATE:   handle_navigate,
        State.ENTER_ROOM: handle_enter_room,
        State.SCAN_ROOM:  handle_scan_room,
        State.DELIVER:    handle_deliver,
        State.RETURN:     handle_return,
    }

    try:
        while state != State.DONE:
            # ── Global safety checks ──
            if is_estop_pressed():
                log("E-STOP PRESSED!")
                stop_all()
                break

            if mission_start_time and (time.time() - mission_start_time > MISSION_TIMEOUT_SEC):
                log("MISSION TIMEOUT! Aborting.")
                stop_all()
                break

            # ── Execute current state ──
            handler = handlers.get(state)
            if handler is None:
                log(f"Unknown state: {state}")
                break

            next_state = handler()
            if next_state != state:
                log(f"Transition: {state} → {next_state}")
            state = next_state

    except KeyboardInterrupt:
        log("Keyboard interrupt — stopping.")
    except Exception as e:
        log(f"UNHANDLED EXCEPTION: {e}")
        import traceback
        traceback.print_exc()
    finally:
        stop_all()
        elapsed = time.time() - mission_start_time if mission_start_time else 0
        print(f"\n{'='*50}")
        print(f"Mission ended in state: {state}")
        print(f"Deliveries completed: {deliveries_completed}/2")
        print(f"Total time: {elapsed:.1f}s")
        print(f"{'='*50}")


if __name__ == "__main__":
    main()
