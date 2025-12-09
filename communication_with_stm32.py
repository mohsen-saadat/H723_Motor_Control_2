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

def parse_reply(packet):
    """
    Expected format: ANGLE,<deg>,CURRENT,<amps>,TORQUE,<Nm>,LOOP_US,<us>,WAIT_US,<us>
    Numeric fields after ANGLE are optional.
    """
    text = packet.decode(errors="ignore").strip()
    parts = text.split(",")
    if len(parts) < 2 or parts[0].upper() != "ANGLE":
        return None
    try:
        angle = float(parts[1])
    except ValueError:
        return None

    current = torque = loop_us = wait_us = None
    idx = 2
    while idx + 1 < len(parts):
        key = parts[idx].strip().upper()
        value = parts[idx + 1].strip()
        idx += 2
        try:
            numeric = float(value)
        except ValueError:
            continue
        if key == "CURRENT":
            current = numeric
        elif key == "TORQUE":
            torque = numeric
        elif key == "LOOP_US":
            loop_us = numeric
        elif key == "WAIT_US":
            wait_us = numeric

    return angle, current, torque, loop_us, wait_us

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

        parsed = parse_reply(data)
        if parsed is None:
            continue
        angle, motor_current, reported_torque, loop_us, wait_us = parsed

        torque_cmd = compute_torque(angle)
        fields = [
            f"angle={angle:.3f} deg",
            f"current={motor_current:.3f} A" if motor_current is not None else "current=—",
            f"mcu_torque={reported_torque:.3f} Nm" if reported_torque is not None else "mcu_torque=—",
            f"loop={loop_us:.1f} us" if loop_us is not None else "loop=—",
            f"wait={wait_us:.1f} us" if wait_us is not None else "wait=—",
            f"cmd={torque_cmd:.3f} Nm",
        ]
        print(", ".join(fields))

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
