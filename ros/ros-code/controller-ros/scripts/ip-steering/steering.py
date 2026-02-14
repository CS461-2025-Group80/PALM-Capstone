#!/usr/bin/env python3
import json, socket, time, math, pygame, sys

# --- CONFIGURATION ---
ROBOT_IP   = "100.120.125.53"   # <-- change to robot's tailscale IP
ROBOT_PORT = 55055           # must match receiver
SEND_HZ    = 50              # send rate
MAX_SPEED  = 1.0             # m/s (tune to taste)

# steering behavior
MAX_YAW_MOVING   = 2.0       # rad/s when driving
MAX_YAW_INPLACE  = 3.5       # rad/s when nearly stopped
STEER_DEADZONE   = 0.1      # ignore tiny stick noise
STEER_EXPO       = 0.65      # 0=linear, 1=very cubic; 0.35–0.55 feels good
STEER_GAIN       = 1.35      # multiply steering after curve

# -----------------------------------------------------------

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def apply_deadzone(x, dz):
    """Remove small joystick noise and rescale."""
    if abs(x) <= dz:
        return 0.0
    return math.copysign((abs(x) - dz) / (1.0 - dz), x)

def expo_curve(x, expo):
    """Add an exponential feel for smoother center response."""
    return (1.0 - expo) * x + expo * (x ** 3)

def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found. Plug in the PXN and retry.")
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using joystick: {js.get_name()} with {js.get_numaxes()} axes, {js.get_numbuttons()} buttons")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    seq = 0
    period = 1.0 / SEND_HZ
    next_t = time.time()

    forward_mode = True  # A = forward, B = reverse

    while True:
        now = time.time()
        if now < next_t:
            time.sleep(next_t - now)
        next_t += period

        for event in pygame.event.get():
            pass

        try:
            raw_steer = js.get_axis(0) if js.get_numaxes() > 0 else 0.0
            raw_steer = -raw_steer  # flip steering direction
            brake_ax = js.get_axis(2) if js.get_numaxes() > 2 else 1.0
            thr_ax   = js.get_axis(5) if js.get_numaxes() > 5 else 1.0
        except pygame.error:
            raw_steer, brake_ax, thr_ax = 0.0, 1.0, 1.0

        # pedals to [0..1]
        accel = (thr_ax + 1.0) / 2.0
        brake = (brake_ax + 1.0) / 2.0

        # button logic
        a_pressed = js.get_button(0) if js.get_numbuttons() > 0 else 0
        b_pressed = js.get_button(1) if js.get_numbuttons() > 1 else 0
        if a_pressed: forward_mode = True
        if b_pressed: forward_mode = False

        # blended speed logic
        drive = accel - brake
        speed = clamp(drive * MAX_SPEED, -MAX_SPEED, MAX_SPEED)
        if not forward_mode:
            speed = -speed
        lin_x = speed

        # --- improved steering response ---
        s = apply_deadzone(raw_steer, STEER_DEADZONE)
        s = expo_curve(s, STEER_EXPO)
        s *= STEER_GAIN
        s = clamp(s, -1.0, 1.0)

        # allow in-place turning and taper yaw with speed
        speed_frac = min(abs(lin_x) / MAX_SPEED, 1.0)
        yaw_limit = MAX_YAW_INPLACE * (1.0 - speed_frac) + MAX_YAW_MOVING * speed_frac
        ang_z = clamp(s * yaw_limit, -yaw_limit, yaw_limit)

        payload = {
            "seq": seq,
            "t": time.time(),
            "lin_x": round(lin_x, 4),
            "ang_z": round(ang_z, 4),
            "accel": round(accel, 3),
            "brake": round(brake, 3),
            "steer": round(raw_steer, 3),
            "fwd": bool(forward_mode),
        }
        seq += 1

        try:
            sock.sendto(json.dumps(payload).encode("utf-8"), (ROBOT_IP, ROBOT_PORT))
        except OSError as e:
            print(f"send error: {e}")

        # lightweight telemetry
        print(f"{payload}", end="\r", flush=True)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting.")


