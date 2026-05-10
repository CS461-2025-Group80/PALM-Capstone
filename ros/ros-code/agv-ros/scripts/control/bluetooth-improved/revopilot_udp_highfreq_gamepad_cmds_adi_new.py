#!/usr/bin/env python3
# ==============================================================================
#                               ENTERPRISE SCRIPT HEADER
# ==============================================================================
# FILE NAME:            revopilot_ros_gamepad_cmds.py
# SUPPORTS SERVICE:     None (run-as-main utility)
# MODULE / COMPONENT:   REVO-Pilot gamepad / wheel teleop for ROS cmd_vel
# SAFETY LEVEL:         CRITICAL (human-in-the-loop teleop; use with caution and safety measures in place)
#
# VERSION:              v1.3.0  (Major.Minor.Patch)
# STATUS:               ACTIVE
#
# AUTHOR:               Dr. Giby Raphael (modifications), aadi (original)
# ORGANIZATION:         REVOBOTS INC.
# CONTACT:              giby@revobots.ai, aditya.raj@revobots.ai
#
# CREATED (LOCAL TZ):   2026-01-05 09:19 (EST)
# LAST MODIFIED (TZ):   2026-04-27 (EST) - Refactored for direct ROS cmd_vel publishing
#
# ------------------------------------------------------------------------------
# CHANGELOG
# ------------------------------------------------------------------------------
# v1.3.0 - 2026-04-27
#   - Refactored from UDP socket communication to ROS cmd_vel topic publishing
#   - Removed network configuration (ROBOT_IPS, ports)
#   - Added rclpy node initialization for ROS 2 (Humble)
#
# v1.2.3 - 2026-03-03
#   - Added Logitech logic
#
# v1.2.2 - 2026-02-26
#   - Added required CLI argument: --robot {SEGWAY,ELEPHANT}
#   - Robot target IP is now selected automatically from --ROBOT
#   - Removed hardcoded ROBOT_IP runtime selection
#
# v1.2.1 - 2026-02-18
#   - Replaced payload field 'orbital_camera' with 'camera'
#   - BTN_X now cycles camera forward: floor → orbital → ai_front → ai_back
#   - BTN_Y now cycles camera backward: ai_back → ai_front → orbital → floor
#
# v1.2.0 - 2026-02-16
#   - Added dynamic speed cycling via unlock sequence (A-B-Y-X)
#   - First unlock sets speed to 1.0 m/s (slow)
#   - Subsequent unlock sequences cycle through: 1.0→2.0→3.0→1.0 (slow→medium→fast)
#   - Added 'speed' field to UDP payload ("slow", "medium", "fast")
#   - Changed --robot-lock parameter to --robot_lock (underscore)
#   - Made --robot_lock parameter case-insensitive (accepts true/True/false/False)
#
# ------------------------------------------------------------------------------
# PURPOSE / OVERVIEW
# ------------------------------------------------------------------------------
# PC-side gamepad / wheel teleop. Reads joystick inputs via pygame, computes
# linear + angular velocities, and publishes ROS 2 Twist messages to cmd_vel
# topic at 50 Hz. Assumes ROS 2 is running and cmd_vel topic is available.
#
# ------------------------------------------------------------------------------
# GAMEPAD CONTROLS (FUNCTIONAL)
# ------------------------------------------------------------------------------
#  - Steering wheel / stick axis  : Steering (angular z)
#  - Throttle pedal / trigger     : Forward speed (lin_x)
#  - Brake pedal / trigger        : Brake (overrides accel)
#  - A button                     : Forward mode
#  - B button                     : Reverse mode
#  - X button                     : Cycle camera forward (floor → orbital → ai_front → ai_back)
#  - Y button                     : Cycle camera backward (ai_back → ai_front → orbital → floor)
#  - Cruise + button              : Increase cruise speed
#  - Cruise - button              : Decrease cruise speed
#  - Head PTZ                     :
#       • Logitech: dedicated head buttons (L/R/U/D)
#       • ANT Pro V3 / Generic: D-pad / hat (L/R/U/D)
#       • PXN V900: D-pad / hat (L/R/U/D)
#  - Robot lock/unlock sequence   :
#       • Unlock: A-B-Y-X within LOCK_SEQUENCE_TIMEOUT
#       • Lock:   X-Y-B-A within LOCK_SEQUENCE_TIMEOUT
#       • Speed cycling (while unlocked):
#           - First unlock sets MAX_SPEED = 1.0 m/s (slow)
#           - Subsequent A-B-Y-X while unlocked cycles speed:
#             1.0 → 2.0 → 3.0 → 1.0 (slow → medium → fast → slow)
#
# NOTE: Button indices differ by controller type; mapping is auto-selected
# based on joystick name. See mapping blocks below.
#
# ------------------------------------------------------------------------------
# INPUTS / OUTPUTS
# ------------------------------------------------------------------------------
# INPUTS:
#   - Joystick / wheel (pygame)
#
# OUTPUTS:
#   - ROS 2 Twist messages to /cmd_vel topic (linear.x, angular.z)
#
# ------------------------------------------------------------------------------
# ROS TOPIC CONFIGURATION
# ------------------------------------------------------------------------------
# Topic: /cmd_vel (geometry_msgs/Twist)
#   - linear.x: forward/backward velocity (m/s)
#   - angular.z: yaw rotation velocity (rad/s)
#   - Publish frequency: 50 Hz
#
# ------------------------------------------------------------------------------
# DEPENDENCIES
# ------------------------------------------------------------------------------
# PYTHON PACKAGES:
#   - pygame
#   - rclpy (ROS 2)
#
# ROS PACKAGES:
#   - geometry_msgs
#
# SYSTEM PACKAGES:
#   - SDL2 (via pygame)
# ==============================================================================
import json, time, math, pygame, sys, argparse
import rclpy
from geometry_msgs.msg import Twist

# ---------------- CONFIGURATION ----------------
SEND_HZ    = 50
SWAP_XY_BUTTONS = True
LOCK_SEQUENCE_TIMEOUT = 2.0  # seconds to complete A,B,Y, (unlock) or X,Y,B,A (lock)
MAX_SPEED_INITIAL = 1.0   # m/s - initial speed after unlock

# Steering behavior
MAX_YAW_MOVING   = 2.0
MAX_YAW_INPLACE  = 3.5
STEER_DEADZONE   = 0.1
STEER_EXPO       = 0.8
STEER_GAIN       = 1.0

# Pedals & cruise
BRAKE_THRESHOLD = 0.2
CRUISE_LEVELS   = [-1.0, -0.6, -0.4, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2, 0.4, 0.6, 1.0]
PEDAL_DEADBAND  = 0.05
# ------------------------------------------------


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def apply_deadzone(x, dz):
    if abs(x) <= dz:
        return 0.0
    return math.copysign((abs(x) - dz) / (1.0 - dz), x)


def expo_curve(x, expo):
    return (1.0 - expo) * x + expo * (x ** 3)

def normalize_pedal(axis, mode):
    if mode == "active_low":
        # +1 rest, -1 pressed
        return clamp((1.0 - axis) / 2.0, 0.0, 1.0)
    else:
        # -1 rest, +1 pressed
        return clamp((axis + 1.0) / 2.0, 0.0, 1.0)


def parse_args():
    parser = argparse.ArgumentParser(
        description="REVO-Pilot gamepad teleop (ROS cmd_vel publisher)."
    )
    parser.add_argument(
        "--robot_lock",
        default="true",
        help="Initial robot_lock state: true or false (default: true).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    initial_robot_lock = args.robot_lock.lower() == "true"
    
    print("\n======================================")
    print(" GAMEPAD TELEOP (ROS cmd_vel)")
    print("======================================")
    print(f" ROBOT_LOCK : {initial_robot_lock}")
    print("======================================\n")
    
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("❌ No joystick found.")
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()

    print("======================================")
    print(" JOYSTICK DETECTED")
    print("======================================")
    print("Name :", js.get_name())
    print("Axes :", js.get_numaxes())
    print("Buttons :", js.get_numbuttons())
    print("Hats :", js.get_numhats())
    print("======================================")

    # ---------------- JOYSTICK MAPPING ----------------

    js_name = js.get_name().lower()
    IS_LOGITECH = False
    
    if "logitech" in js_name or "g29" in js_name or "g920" in js_name or "g923" in js_name:
        print("[MAPPING] Using LOGITECH mapping")
        
        IS_LOGITECH = True
        
        
        AXIS_STEER  = 0
        AXIS_THROTTLE = 2
        AXIS_BRAKE  = 3
        BTN_A = 0
        BTN_B = 1
        BTN_Y = 2 #top camera (eth)
        BTN_X = 3 #bottom camera (usb)
        BTN_CRUISE_UP = 4
        BTN_CRUISE_DOWN = 5

        PEDAL_MODE = "active_low"   # +1 rest, -1 pressed

        # --- HEAD (Logitech buttons) ---
        # Logitech uses HAT, not buttons
        BTN_HEAD_LEFT  = None
        BTN_HEAD_RIGHT = None
        BTN_HEAD_UP    = None
        BTN_HEAD_DOWN  = None
        
        # --- Logitech HAT PTZ state ---
        prev_hat = (0, 0)
        hat_hold_start = 0.0
        HAT_STEP_DELAY = 0.25   # seconds between step moves while holding
    
    elif "tgz controller" == js_name:
        print("[MAPPING] Using MyAGV Bluetooth TGZ Controller mapping")

        AXIS_STEER = 2 # right joystick left and right
        AXIS_THROTTLE = 4 # right trigger
        AXIS_BRAKE = 5 # left trigger
        BTN_A = 0 # A
        BTN_B = 1 # B
        BTN_X = 3 # X
        BTN_Y = 4 # Y

        BTN_CRUISE_UP = 7 # right bumper
        BTN_CRUISE_DOWN = 6 # left bumper

        PEDAL_MODE = "active_high"   # -1 rest, +1 pressed

        # HAT is used, a pair of two integers represents the D-pad: (left/right press, forward/backward press)
        BTN_HEAD_LEFT  = None
        BTN_HEAD_RIGHT = None
        BTN_HEAD_UP    = None
        BTN_HEAD_DOWN  = None

        prev_hat = (0, 0)
        hat_hold_start = 0.0
        HAT_STEP_DELAY = 0.25   # seconds between step moves while holding

    elif "x-box" in js_name or "generic" in js_name: # this is the 7k cheaper one. 
        print("[MAPPING] Using ANT Pro V3 mapping")
        AXIS_STEER  = 0
        AXIS_THROTTLE = 5
        AXIS_BRAKE  = 2
        BTN_A = 0
        BTN_B = 1
        BTN_Y = 2 #top camera (eth)
        BTN_X = 3 #bottom camera (usb)
        BTN_CRUISE_UP = 5 # button on right behind the wheel
        BTN_CRUISE_DOWN = 4 # button on left behind the wheel
        #BTN_MIC_TOGGLE = 5 # right paddle behind the wheel
        #BTN_HORN_TOGGLE = 4 # left paddle behind the wheel

        PEDAL_MODE = "active_high"  # -1 rest, +1 pressed

        # --- HEAD : not buttons, dedicated hat axis ---

    elif "litestar" in js_name and "v900" in js_name:
        print("[MAPPING] Using PXN V900 mapping")
        AXIS_STEER  = 0 #default 0
        AXIS_THROTTLE = 2 #default -32767
        AXIS_BRAKE  = 5 #default -32767
        BTN_A = 0
        BTN_B = 1
        BTN_Y = 2 #top camera (eth)
        BTN_X = 3 #bottom camera (usb)
        BTN_CRUISE_UP = 14
        BTN_CRUISE_DOWN = 13

        PEDAL_MODE = "active_high"   # -1 rest, +1 pressed

        # --- HEAD : not buttons, dedicated hat axis ---

    elif "pxn" in js_name and "v900" in js_name:
        print("[MAPPING] Using PXN V900 mapping")
        AXIS_STEER  = 0 #default 0
        AXIS_THROTTLE = 5 #default -32767
        AXIS_BRAKE  = 2 #default -32767
        BTN_A = 0
        BTN_B = 1
        BTN_Y = 2 #top camera (eth)
        BTN_X = 3 #bottom camera (usb)
        BTN_CRUISE_UP = 5
        BTN_CRUISE_DOWN = 4

        PEDAL_MODE = "active_high"   # -1 rest, +1 pressed

        # --- HEAD : not buttons, dedicated hat axis ---
        
    elif "wired" in js_name : # this is the 7k cheaper one. 
        print("[MAPPING] Using ANT Pro V3 mapping")
        AXIS_STEER  = 0
        AXIS_THROTTLE = 5
        AXIS_BRAKE  = 4
        BTN_A = 0
        BTN_B = 1
        BTN_Y = 2 #top camera (eth)
        BTN_X = 3 #bottom camera (usb)
        BTN_CRUISE_UP = 5 # button on right behind the wheel
        BTN_CRUISE_DOWN = 4 # button on left behind the wheel
        #BTN_MIC_TOGGLE = 5 # right paddle behind the wheel
        #BTN_HORN_TOGGLE = 4 # left paddle behind the wheel

        PEDAL_MODE = "active_high"  # -1 rest, +1 pressed

        # --- HEAD : not buttons, dedicated hat axis ---
        
    else:
        print(f"[WARN] Unknown joystick '{js.get_name()}'. Falling back to LOGITECH mapping.")
        AXIS_STEER  = 0
        AXIS_THROTTLE = 2
        AXIS_BRAKE  = 3
        BTN_A = 0
        BTN_B = 1
        BTN_Y = 2 #top camera (eth)
        BTN_X = 3 #bottom camera (usb)
        BTN_CRUISE_UP = 4
        BTN_CRUISE_DOWN = 5

        PEDAL_MODE = "active_low"   # +1 rest, -1 pressed

        BTN_HEAD_LEFT  = None
        BTN_HEAD_RIGHT = None
        BTN_HEAD_UP    = None
        BTN_HEAD_DOWN  = None

    print(f"[JOYSTICK] {js.get_name()} → "
      f"steer={AXIS_STEER}, throttle={AXIS_THROTTLE}, brake={AXIS_BRAKE}, "
      f"cruise+={BTN_CRUISE_UP}, cruise-={BTN_CRUISE_DOWN}, pedal_mode={PEDAL_MODE}")
    

    # Initialize ROS node
    rclpy.init()
    node = rclpy.create_node('gamepad_teleop')
    cmd_vel_pub = node.create_publisher(Twist, 'cmd_vel', 10)

    seq = 0
    period = 1.0 / SEND_HZ
    next_t = time.time()

    forward_mode = True
    cruise_zero_idx = CRUISE_LEVELS.index(0.0)
    cruise_level_idx = cruise_zero_idx
    cruise_speed = CRUISE_LEVELS[cruise_level_idx]
    camera_modes = ["floor", "orbital", "ai_front", "ai_back"]
    camera_index = 1  # default to "orbital"
    camera_mode = camera_modes[camera_index]
    robot_lock = initial_robot_lock
    max_speed = MAX_SPEED_INITIAL  # dynamic speed limit
    speed_level = 1  # 1=slow, 2=medium, 3=fast

    # --- lock sequence state ---
    lock_seq = []  # list of (button_char, timestamp)
    seq_start_state = None  # (forward_mode, camera_mode) snapshot at sequence start
    sequence_active = False  # suppress camera changes once AB/XY prefix detected
    prev_a = 0
    prev_b = 0
    prev_y = 0
    prev_x = 0

    # --- edge detection state ---
    prev_cruise_up = 0
    prev_cruise_down = 0

    while True:
        now = time.time()
        if now < next_t:
            time.sleep(next_t - now)
        next_t += period

        pygame.event.pump()

        head = "center"

        # -------- AXES --------
        try:
            raw_steer = -js.get_axis(AXIS_STEER)   # steering
            thr_ax   = js.get_axis(AXIS_THROTTLE)    # throttle (active-low)
            brake_ax = js.get_axis(AXIS_BRAKE)    # brake (active-low)
           
            #print("raw_steer:", raw_steer, "thr_ax:", thr_ax, "brake_ax:", brake_ax, end="\r", flush=True)
        except pygame.error:
            raw_steer, thr_ax, brake_ax = 0.0, 1.0, 1.0

        # convert to accel and brake [0.0, 1.0], use joystick mode
        #accel = (1.0 - thr_ax) / 2.0
        #brake = (1.0 - brake_ax) / 2.0
        accel = normalize_pedal(thr_ax, PEDAL_MODE)
        brake = normalize_pedal(brake_ax, PEDAL_MODE)

        # -------- BUTTONS --------
        a_pressed = js.get_button(BTN_A) if js.get_numbuttons() > BTN_A else 0
        b_pressed = js.get_button(BTN_B) if js.get_numbuttons() > BTN_B else 0
        
        y_pressed = js.get_button(BTN_Y) if js.get_numbuttons() > BTN_Y else 0
        x_pressed = js.get_button(BTN_X) if js.get_numbuttons() > BTN_X else 0

        if SWAP_XY_BUTTONS:
            y_pressed, x_pressed = x_pressed, y_pressed # orbital cam is Y by default, but some controllers have X/Y swapped, so add option to swap them in software

        # --- lock/unlock sequence (edge-triggered) ---
        pre_forward_mode = forward_mode
        pre_camera_mode = camera_mode
        now_t = time.time()

        a_edge = a_pressed and not prev_a
        b_edge = b_pressed and not prev_b
        y_edge = y_pressed and not prev_y
        x_edge = x_pressed and not prev_x

        if a_edge:
            if not lock_seq and seq_start_state is None:
                seq_start_state = (pre_forward_mode, pre_camera_mode)
            lock_seq.append(("A", now_t))
        if b_edge:
            if not lock_seq and seq_start_state is None:
                seq_start_state = (pre_forward_mode, pre_camera_mode)
            lock_seq.append(("B", now_t))
        if y_edge:
            if not lock_seq and seq_start_state is None:
                seq_start_state = (pre_forward_mode, pre_camera_mode)
            lock_seq.append(("Y", now_t))
        if x_edge:
            if not lock_seq and seq_start_state is None:
                seq_start_state = (pre_forward_mode, pre_camera_mode)
            lock_seq.append(("X", now_t))

        prev_a = a_pressed
        prev_b = b_pressed
        prev_y = y_pressed
        prev_x = x_pressed

        # keep only recent presses
        lock_seq = [(k, t) for (k, t) in lock_seq if now_t - t <= LOCK_SEQUENCE_TIMEOUT]


        if not sequence_active and len(lock_seq) >= 2:
            first2 = "".join([k for (k, _) in lock_seq[:2]])
            if first2 in ("AB", "XY"):
                sequence_active = True

        sequence_matched = False
        if len(lock_seq) >= 4:
            last4 = lock_seq[-4:]
            seq_str = "".join([k for (k, _) in last4])
            span = last4[-1][1] - last4[0][1]
            if span <= LOCK_SEQUENCE_TIMEOUT:
                if seq_str == "ABYX":
                    if robot_lock:
                        # First unlock: set to slow speed
                        robot_lock = False
                        max_speed = 1.0
                        speed_level = 1
                    else:
                        # Already unlocked: cycle speed 1→2→3→1
                        speed_level += 1
                        if speed_level > 3:
                            speed_level = 1
                        max_speed = float(speed_level)
                    lock_seq = []
                    sequence_matched = True
                elif seq_str == "XYBA":
                    robot_lock = True
                    lock_seq = []
                    sequence_matched = True

        if sequence_matched:
            if seq_start_state is not None:
                forward_mode, camera_mode = seq_start_state
            else:
                forward_mode, camera_mode = pre_forward_mode, pre_camera_mode
            seq_start_state = None
            sequence_active = False
        elif not lock_seq and seq_start_state is not None:
            seq_start_state = None
            sequence_active = False

        if not sequence_matched:
            if a_pressed:
                forward_mode = True
            if b_pressed:
                forward_mode = False

            if not sequence_active:
                if x_edge:
                    camera_index = (camera_index + 1) % len(camera_modes)
                    camera_mode = camera_modes[camera_index]
                if y_edge:
                    camera_index = (camera_index - 1) % len(camera_modes)
                    camera_mode = camera_modes[camera_index]

        cruise_up   = js.get_button(BTN_CRUISE_UP) if js.get_numbuttons() > BTN_CRUISE_UP else 0
        cruise_down = js.get_button(BTN_CRUISE_DOWN) if js.get_numbuttons() > BTN_CRUISE_DOWN else 0

        
        # -------- LINEAR SPEED LOGIC --------
        both_cruise_pressed = bool(cruise_up and cruise_down)
        brake_active = (brake > BRAKE_THRESHOLD) or both_cruise_pressed

        pedal_speed = accel * max_speed
        if pedal_speed <= PEDAL_DEADBAND:
            pedal_speed = 0.0

        # ---- CRUISE CONTROL (EDGE-TRIGGERED, DISCRETE LEVELS) ----
        if both_cruise_pressed:
            cruise_level_idx = cruise_zero_idx
        else:
            if cruise_up and not prev_cruise_up:
                cruise_level_idx = min(cruise_level_idx + 1, len(CRUISE_LEVELS) - 1)

            if cruise_down and not prev_cruise_down:
                cruise_level_idx = max(cruise_level_idx - 1, 0)

        cruise_speed = CRUISE_LEVELS[cruise_level_idx]

        prev_cruise_up = cruise_up
        prev_cruise_down = cruise_down

        cruise_abs_max = min(1.0, max_speed)
        cruise_speed = clamp(cruise_speed, -cruise_abs_max, cruise_abs_max)

        #print("pedal_speed:", round(pedal_speed,2), "cruise_up:", cruise_up, "cruise_down:", cruise_down, "cruise_speed:", round(cruise_speed,2), end="\r", flush=True)

        # Priority: brake > pedal > cruise
        if brake_active:
            lin_x = 0.0
            cruise_speed = 0.0
            cruise_level_idx = cruise_zero_idx
        elif pedal_speed > 0.0:
            lin_x = pedal_speed
            cruise_speed = 0.0
            cruise_level_idx = cruise_zero_idx
        else:
            lin_x = cruise_speed

        if not forward_mode:
            lin_x = -lin_x
            cruise_speed = 0.0

 
         # --- HEAD: LOGITECH (buttons) ---
        try:
            # --- LOGITECH: HAT-based PTZ ---
            if IS_LOGITECH and js.get_numhats() > 0:
                hx, hy = js.get_hat(0)
        
                # EDGE detect (step movement)
                if (hx, hy) != prev_hat:
                    if hy == 1:
                        head = "up"
                    elif hy == -1:
                        head = "down"
                    elif hx == 1:
                        head = "right"
                    elif hx == -1:
                        head = "left"
                    hat_hold_start = now_t
        
                # HOLD detect (continuous stepping)
                elif (hx, hy) != (0, 0):
                    if now_t - hat_hold_start >= HAT_STEP_DELAY:
                        if hy == 1:
                            head = "up"
                        elif hy == -1:
                            head = "down"
                        elif hx == 1:
                            head = "right"
                        elif hx == -1:
                            head = "left"
                        hat_hold_start = now_t
        
                prev_hat = (hx, hy)

            # --- HEAD: PXN V900 (axes 7/8) ---
            #elif ("litestar" in js_name) or ("v900" in js_name):
            elif "v900" in js_name:
                head = "center"
                if js.get_numhats() > 0:
                    hx, hy = js.get_hat(0)  # typically: left=-1 right=+1, up=+1 down=-1

                    if hx < 0:
                        head = "left"
                    elif hx > 0:
                        head = "right"
                    elif hy > 0:
                        head = "up"
                    elif hy < 0:
                        head = "down"
        
        except pygame.error:
            head = "center"

        

        # -------- STEERING (UNCHANGED) --------
        s = apply_deadzone(raw_steer, STEER_DEADZONE)
        s = expo_curve(s, STEER_EXPO)
        s *= STEER_GAIN
        s = clamp(s, -1.0, 1.0)

        speed_frac = min(abs(lin_x) / max_speed, 1.0) if max_speed > 0 else 0.0
        yaw_limit = MAX_YAW_INPLACE * (1.0 - speed_frac) + MAX_YAW_MOVING * speed_frac
        ang_z = clamp(s * yaw_limit, -yaw_limit, yaw_limit)

        if both_cruise_pressed:
            ang_z = 0.0

        # -------- PAYLOAD --------
        speed_label = {1: "slow", 2: "medium", 3: "fast"}.get(speed_level, "slow")
        payload = {
            "seq": seq,
            "t": time.time(),
            "lin_x": round(lin_x, 4),
            "ang_z": round(ang_z, 4),
            "accel": round(accel, 3),
            "brake": round(brake, 3),
            "cruise": round(cruise_speed, 3),
            "fwd": forward_mode,
            "camera": camera_mode,
            "robot_lock": robot_lock,
            "head": head,
            "speed": speed_label,
        }
        seq += 1

        # Publish Twist message to cmd_vel topic
        try:
            twist = Twist()
            twist.linear.x = lin_x
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = ang_z
            
            cmd_vel_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0)

        except Exception as e:
            print(f"publish error: {e}")

        print(
            f"#{seq:06d}  "
            f"SPD:{speed_label:<6}  "
            f"LIN:{lin_x:+.2f}  "
            f"YAW:{ang_z:+.2f}  "
            f"ACC:{accel:.2f}  BRK:{brake:.2f}  CRU:{cruise_speed:.2f}  "
            f"CAM:{camera_mode}  "
            f"HEAD:{head:<6}  "
            f"LOCK:{'YES' if robot_lock else 'NO '}",
            end="\r",
            flush=True,
        )

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Exiting cleanly.")
        rclpy.shutdown()
