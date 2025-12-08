#!/usr/bin/env python3
import socket
import time

STM32_IP       = "192.168.50.123"
STM32_PORT     = 11000
LOOP_HZ        = 500.0
PERIOD         = 1.0 / LOOP_HZ

# Match the constants in main.cpp
SPRING_COEFF   = 0.00002
DESIRED_ANGLE  = 0.0
TORQUE_LIMIT   = 8.0

def clamp(val, limit):
    if val >  limit: return limit
    if val < -limit: return -limit
    return val

def compute_torque(angle_deg):
    error = angle_deg - DESIRED_ANGLE
    torque = -SPRING_COEFF * error * abs(error)
    return clamp(torque, TORQUE_LIMIT)

def parse_angle_current(packet):
    """
    Expected format: ANGLE,<deg>[,CURRENT,<amps>]
    Returns (angle_deg, current_amps or None)
    """
    text = packet.decode(errors="ignore").strip()
    parts = text.split(",")
    if len(parts) < 2 or parts[0].upper() != "ANGLE":
        return None
    try:
        angle = float(parts[1])
    except ValueError:
        return None

    current = None
    if len(parts) >= 4 and parts[2].upper() == "CURRENT":
        try:
            current = float(parts[3])
        except ValueError:
            current = None

    return angle, current

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.2)   # if we miss a reply, resend the last torque

    torque_cmd = 0.0
    next_tick = time.perf_counter()

    def send_torque(value):
        msg = f"TORQUE,{value:.3f}".encode("ascii")
        sock.sendto(msg, (STM32_IP, STM32_PORT))

    # Kick-start the loop with a zero torque request
    send_torque(torque_cmd)

    while True:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            # keep sending the last command if the MCU missed it
            send_torque(torque_cmd)
            continue

        parsed = parse_angle_current(data)
        if parsed is None:
            continue
        angle, motor_current = parsed

        torque_cmd = compute_torque(angle)
        if motor_current is not None:
            print(f"angle={angle:.3f} deg, current={motor_current:.3f} A -> torque={torque_cmd:.3f} Nm")
        else:
            print(f"angle={angle:.3f} deg -> torque={torque_cmd:.3f} Nm")

        # Keep the loop close to 100 Hz
        next_tick += PERIOD
        delay = next_tick - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.perf_counter()

        send_torque(torque_cmd)

if __name__ == "__main__":
    main()
