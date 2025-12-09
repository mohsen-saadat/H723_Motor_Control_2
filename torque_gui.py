#!/usr/bin/env python3
"""
High-rate UDP client with Tk GUI for STM32 virtual-spring experiment.

Networking thread:
  - Runs at ~1 kHz command rate.
  - Receives ANGLE/CURRENT packets, computes a new torque command, and
    streams packets to the MCU.

GUI thread:
  - Updates ~200 Hz via Tkinter.
  - Draws three arrows (angle, current, torque) plus numeric readouts.
"""

import math
import socket
import threading
import time
from queue import Empty, Queue
import tkinter as tk

# --- Networking parameters -------------------------------------------------
STM32_IP = "192.168.50.123"
STM32_PORT = 11000

COMM_HZ = 1000.0             # network worker target frequency
COMM_PERIOD = 1.0 / COMM_HZ
GUI_HZ = 200.0               # GUI refresh
GUI_PERIOD_MS = int(1000.0 / GUI_HZ)

# --- Control law constants (match firmware) --------------------------------
SPRING_COEFF = 0.00001
DESIRED_ANGLE_DEG = 0.0
TORQUE_LIMIT_NM = 8.0

# --- Shared queues / events ------------------------------------------------
state_queue: "Queue[tuple[float, float, float, float, float, float]]" = Queue()
stop_event = threading.Event()


def clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def compute_torque(angle_deg: float) -> float:
    error = angle_deg - DESIRED_ANGLE_DEG
    torque = -SPRING_COEFF * error * abs(error)
    return clamp(torque, TORQUE_LIMIT_NM)


def parse_reply(packet: bytes) -> tuple[float, float, float, float, float] | None:
    """
    Expected format: ANGLE,<deg>,CURRENT,<amps>,TORQUE,<Nm>,LOOP_US,<us>,WAIT_US,<us>
    Returns tuple (angle, current, torque, loop_us, wait_us); missing fields default to 0.
    """
    text = packet.decode(errors="ignore").strip()
    parts = text.split(",")
    if len(parts) < 2 or parts[0].upper() != "ANGLE":
        return None
    try:
        angle = float(parts[1])
    except ValueError:
        return None

    current = torque = loop_us = wait_us = 0.0
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


def networking_worker():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.001)

    torque_cmd = 0.0
    next_send = time.perf_counter()

    def send_torque(value: float):
        msg = f"TORQUE,{value:.3f}".encode("ascii")
        try:
            sock.sendto(msg, (STM32_IP, STM32_PORT))
        except OSError:
            pass

    send_torque(torque_cmd)

    while not stop_event.is_set():
        now = time.perf_counter()
        if now >= next_send:
            send_torque(torque_cmd)
            next_send += COMM_PERIOD

        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break

        parsed = parse_reply(data)
        if not parsed:
            continue

        angle_deg, motor_current, reported_torque, loop_us, wait_us = parsed
        torque_cmd = compute_torque(angle_deg)

        try:
            # keep only the latest sample to avoid GUI lag
            while state_queue.qsize() > 4:
                state_queue.get_nowait()
        except Empty:
            pass

        state_queue.put((time.time(), angle_deg, motor_current, reported_torque, loop_us, wait_us))

    sock.close()


# --- GUI helpers -----------------------------------------------------------
class GaugeView:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("STM32 Torque Control Monitor")

        self.gauges = []
        labels = [("Angle", "#1a73e8"), ("Current", "#34a853"), ("Torque", "#ea4335")]
        for idx, (title, color) in enumerate(labels):
            canvas = tk.Canvas(
                root,
                width=200,
                height=200,
                bg="white",
                highlightthickness=1,
                highlightbackground="#cccccc",
            )
            canvas.grid(row=0, column=idx, padx=10, pady=10)
            center = 100
            radius = 80
            canvas.create_oval(center - radius,
                               center - radius,
                               center + radius,
                               center + radius,
                               outline="#444")
            arrow = canvas.create_line(center, center, center + radius, center,
                                       width=4, fill=color, arrow="last")
            canvas.create_text(center, 15, text=title, font=("Arial", 12, "bold"))
            self.gauges.append((canvas, arrow, center, radius))

        self.angle_var = tk.StringVar(value="Angle: — deg")
        self.current_var = tk.StringVar(value="Current: — A")
        self.torque_var = tk.StringVar(value="Torque cmd: — Nm")
        self.rate_var = tk.StringVar(value="Update rate: — Hz")
        self.loop_var = tk.StringVar(value="Loop time: — µs")
        self.wait_var = tk.StringVar(value="Wait time: — µs")

        tk.Label(root, textvariable=self.angle_var).grid(row=1, column=0, padx=8, sticky="w")
        tk.Label(root, textvariable=self.current_var).grid(row=1, column=1, padx=8, sticky="w")
        tk.Label(root, textvariable=self.torque_var).grid(row=1, column=2, padx=8, sticky="w")
        tk.Label(root, textvariable=self.rate_var).grid(row=2, column=0, columnspan=3, pady=(4, 2))
        tk.Label(root, textvariable=self.loop_var).grid(row=3, column=0, columnspan=3, pady=(0, 2))
        tk.Label(root, textvariable=self.wait_var).grid(row=4, column=0, columnspan=3, pady=(0, 12))

        self.last_timestamp = None

    def update_arrows(self, angle_deg: float, motor_current: float, torque_actual: float):
        self._set_polar_arrow(0, angle_deg)

        current_scale = max(min(motor_current / 16.0, 1.0), -1.0)
        current_angle = current_scale * 360.0
        self._set_polar_arrow(1, current_angle)

        torque_scale = max(min(torque_actual / 4.0, 1.0), -1.0)
        torque_angle = torque_scale * 360.0
        self._set_polar_arrow(2, torque_angle)

        self.angle_var.set(f"Angle: {angle_deg:7.3f} deg")
        self.current_var.set(f"Current: {motor_current:6.3f} A")
        self.torque_var.set(f"Torque cmd: {torque_actual:6.3f} Nm")

    def update_rate(self, timestamp: float):
        if self.last_timestamp is not None:
            dt = timestamp - self.last_timestamp
            if dt > 0:
                rate = 1.0 / dt
                self.rate_var.set(f"Update rate: {rate:6.1f} Hz")
        self.last_timestamp = timestamp

    def update_loop_time(self, loop_time_us: float):
        self.loop_var.set(f"Loop time: {loop_time_us:7.1f} µs")

    def update_wait_time(self, wait_time_us: float):
        self.wait_var.set(f"Wait time: {wait_time_us:7.1f} µs")

    def _set_polar_arrow(self, index: int, angle_deg: float):
        canvas, arrow_id, center, radius = self.gauges[index]
        theta = math.radians(angle_deg)
        x = center + radius * math.cos(theta)
        y = center - radius * math.sin(theta)
        canvas.coords(arrow_id, center, center, x, y)


def gui_loop():
    root = tk.Tk()
    view = GaugeView(root)

    def pump_queue():
        latest = None
        while True:
            try:
                latest = state_queue.get_nowait()
            except Empty:
                break
        if latest:
            timestamp, angle_deg, motor_current, torque_actual, loop_us, wait_us = latest
            view.update_arrows(angle_deg, motor_current, torque_actual)
            view.update_rate(timestamp)
            view.update_loop_time(loop_us)
            view.update_wait_time(wait_us)
        root.after(GUI_PERIOD_MS, pump_queue)

    def on_close():
        stop_event.set()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(GUI_PERIOD_MS, pump_queue)
    root.mainloop()


def main():
    worker = threading.Thread(target=networking_worker, daemon=True)
    worker.start()
    try:
        gui_loop()
    finally:
        stop_event.set()
        worker.join(timeout=1.0)


if __name__ == "__main__":
    main()
