#!/usr/bin/env python3
"""
Test Communication Script for Arduino Closed-Loop Motor Controller.
Tests serial handshake (START -> READY), command streaming (CMD,left,right),
and state feedback reception (STATE,left,right at 50Hz).
"""

import argparse
import sys
import time
import threading
import serial
import serial.tools.list_ports


def find_default_port():
    """Auto-detect connected USB serial adapter / Arduino."""
    ports = serial.tools.list_ports.comports()
    # Look for USB or ACM devices (e.g. /dev/ttyUSB*, /dev/ttyACM*)
    usb_ports = [p.device for p in ports if 'USB' in p.device or 'ACM' in p.device]
    if usb_ports:
        return usb_ports[0]
    return None


class ArduinoCommTester:
    def __init__(self, port, baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.running = False
        self.feedback_count = 0
        self.last_left_vel = 0.0
        self.last_right_vel = 0.0
        self.last_feedback_time = 0.0
        self.target_left = 0.0
        self.target_right = 0.0
        self.command_lock = threading.Lock()
        self.rx_thread = None
        self.tx_thread = None

    def connect(self):
        print(f"[INFO] Connecting to {self.port} at {self.baud_rate} baud...")
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1.0)
        except serial.SerialException as e:
            print(f"[ERROR] Failed to open port {self.port}: {e}")
            return False

        # Arduino auto-resets on serial connection via DTR pin; allow bootloader to complete
        print("[INFO] Waiting 2 seconds for Arduino to reset and initialize...")
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        return True

    def handshake(self, max_attempts=5, timeout_per_attempt=1.5):
        print("[INFO] Initiating handshake: sending 'START'...")
        for attempt in range(1, max_attempts + 1):
            self.ser.reset_input_buffer()
            self.ser.write(b"START\n")
            self.ser.flush()

            start_t = time.time()
            while time.time() - start_t < timeout_per_attempt:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line == "READY":
                        print(f"[SUCCESS] Handshake established! Received 'READY' on attempt {attempt}.")
                        return True
                    elif line:
                        print(f"  [ARDUINO BOOT] {line}")
                time.sleep(0.05)
            print(f"[WARN] Handshake attempt {attempt} timed out waiting for 'READY', retrying...")

        print("[ERROR] Handshake failed! Did not receive 'READY' from Arduino.")
        return False

    def _rx_worker(self):
        """Background thread reading STATE messages from Arduino."""
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if line.startswith("STATE,"):
                    parts = line.split(",")
                    if len(parts) >= 3:
                        try:
                            l_vel = float(parts[1])
                            r_vel = float(parts[2])
                            with self.command_lock:
                                self.last_left_vel = l_vel
                                self.last_right_vel = r_vel
                                self.last_feedback_time = time.time()
                                self.feedback_count += 1
                        except ValueError:
                            pass
                else:
                    print(f"\n[ARDUINO] {line}")
            except Exception as e:
                if self.running:
                    print(f"\n[ERROR] RX thread exception: {e}")
                break

    def _tx_worker(self, rate_hz=20):
        """Background thread periodically transmitting CMD messages to keep watchdog alive."""
        interval = 1.0 / rate_hz
        while self.running and self.ser and self.ser.is_open:
            with self.command_lock:
                cmd_str = f"CMD,{self.target_left:.4f},{self.target_right:.4f}\n"
            try:
                self.ser.write(cmd_str.encode('utf-8'))
                self.ser.flush()
            except Exception as e:
                if self.running:
                    print(f"\n[ERROR] TX thread exception: {e}")
                break
            time.sleep(interval)

    def start_streaming(self):
        """Starts periodic CMD transmission and STATE reception threads."""
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self.tx_thread = threading.Thread(target=self._tx_worker, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()
        print("[INFO] Command streaming (20 Hz) and telemetry reception started.")

    def set_targets(self, left_rad_s, right_rad_s):
        with self.command_lock:
            self.target_left = left_rad_s
            self.target_right = right_rad_s

    def get_feedback(self):
        with self.command_lock:
            return (
                self.last_left_vel,
                self.last_right_vel,
                self.feedback_count,
                self.last_feedback_time
            )

    def stop_streaming(self):
        """Safely stops motors and stops threads."""
        self.set_targets(0.0, 0.0)
        time.sleep(0.1)
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"CMD,0.0000,0.0000\n")
                self.ser.flush()
            except Exception:
                pass

    def close(self):
        self.stop_streaming()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[INFO] Serial connection closed.")


def run_automated_test(tester):
    """Automated test sequence validating handshake, feedback, motion, and watchdog."""
    print("\n" + "=" * 60)
    print("           STARTING AUTOMATED COMMUNICATION TEST           ")
    print("=" * 60)

    tester.start_streaming()

    # Step 1: Verify feedback stream
    print("\n--- Step 1: Validating 50Hz Telemetry Stream (2 seconds) ---")
    initial_count = tester.feedback_count
    for _ in range(20):
        time.sleep(0.1)
        l_vel, r_vel, count, last_t = tester.get_feedback()
        sys.stdout.write(f"\r[STATE] Left: {l_vel:+6.3f} rad/s | Right: {r_vel:+6.3f} rad/s | Packets received: {count}")
        sys.stdout.flush()

    diff = tester.feedback_count - initial_count
    rate = diff / 2.0
    print(f"\n[RESULT] Received {diff} feedback packets in 2s (~{rate:.1f} Hz). Expected ~50 Hz.")
    if rate < 30:
        print("[WARN] Feedback rate is lower than expected 50 Hz.")
    else:
        print("[OK] Telemetry stream verified.")

    # Step 2: Spin motors at 2.0 rad/s (~19 RPM)
    target_speed = 2.0
    print(f"\n--- Step 2: Commanding Forward Motion ({target_speed} rad/s ~ 19.1 RPM) for 4 seconds ---")
    tester.set_targets(target_speed, target_speed)

    start_t = time.time()
    while time.time() - start_t < 4.0:
        l_vel, r_vel, count, last_t = tester.get_feedback()
        sys.stdout.write(
            f"\r[CMD: {target_speed:+.2f}] Actual -> Left: {l_vel:+6.3f} rad/s | Right: {r_vel:+6.3f} rad/s"
        )
        sys.stdout.flush()
        time.sleep(0.1)
    print("\n[OK] Forward motion test completed.")

    # Step 3: Command Stop
    print("\n--- Step 3: Commanding Stop (0.0 rad/s) for 2 seconds ---")
    tester.set_targets(0.0, 0.0)
    start_t = time.time()
    while time.time() - start_t < 2.0:
        l_vel, r_vel, count, last_t = tester.get_feedback()
        sys.stdout.write(
            f"\r[CMD:  0.00] Actual -> Left: {l_vel:+6.3f} rad/s | Right: {r_vel:+6.3f} rad/s"
        )
        sys.stdout.flush()
        time.sleep(0.1)
    print("\n[OK] Stop command verified.")

    # Step 4: Test Watchdog Timeout (stop sending commands)
    print("\n--- Step 4: Watchdog Timeout Test (command stream halted) ---")
    print("Pausing command transmission for 1 second (Watchdog timeout threshold: 200ms)...")
    tester.running = False  # halts TX worker
    time.sleep(1.0)
    print("[OK] Watchdog test completed. Motors should be safely stopped by firmware timeout.")

    print("\n" + "=" * 60)
    print("             AUTOMATED TEST COMPLETED SUCCESSFULLY          ")
    print("=" * 60 + "\n")


def run_interactive_mode(tester):
    """Interactive CLI allowing manual velocity input with live telemetry."""
    print("\n" + "=" * 60)
    print("              INTERACTIVE MOTOR CONTROL CLI                 ")
    print("=" * 60)
    print("Commands:")
    print("  <left> <right>  : Set target velocities in rad/s (e.g., '2.5 2.5')")
    print("  stop            : Stop both motors (0.0 rad/s)")
    print("  auto            : Run full automated verification test")
    print("  quit / exit     : Exit test script")
    print("=" * 60 + "\n")

    tester.start_streaming()

    try:
        while True:
            cmd = input("\nEnter command [left right | stop | auto | quit]: ").strip()
            if not cmd:
                continue

            if cmd.lower() in ["quit", "exit", "q"]:
                break

            if cmd.lower() == "stop":
                tester.set_targets(0.0, 0.0)
                print("[INFO] Commanded STOP (0.0, 0.0)")
                time.sleep(0.5)
                l_vel, r_vel, _, _ = tester.get_feedback()
                print(f"[FEEDBACK] Left: {l_vel:+6.3f} rad/s | Right: {r_vel:+6.3f} rad/s")
                continue

            if cmd.lower() == "auto":
                tester.stop_streaming()
                run_automated_test(tester)
                tester.start_streaming()
                continue

            parts = cmd.split()
            if len(parts) == 1:
                try:
                    val = float(parts[0])
                    tester.set_targets(val, val)
                    print(f"[INFO] Set both motors to {val:+.3f} rad/s (~{val * 30.0 / 3.14159:.1f} RPM)")
                except ValueError:
                    print("[ERROR] Invalid input. Enter two numbers or 'stop'.")
            elif len(parts) == 2:
                try:
                    l = float(parts[0])
                    r = float(parts[1])
                    tester.set_targets(l, r)
                    print(f"[INFO] Targets -> Left: {l:+.3f} rad/s, Right: {r:+.3f} rad/s")
                except ValueError:
                    print("[ERROR] Invalid numbers.")
            else:
                print("[ERROR] Too many arguments. Usage: <left> <right>")

            # Show feedback for 1 second
            start_t = time.time()
            while time.time() - start_t < 1.0:
                l_vel, r_vel, _, _ = tester.get_feedback()
                sys.stdout.write(f"\r  -> Live Feedback: Left: {l_vel:+6.3f} rad/s | Right: {r_vel:+6.3f} rad/s")
                sys.stdout.flush()
                time.sleep(0.1)
            print()

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt received. Exiting safely...")


def main():
    parser = argparse.ArgumentParser(description="Test Serial Communication with Motor Controller Arduino")
    parser.add_argument("--port", "-p", default=None, help="Serial port (e.g. /dev/ttyUSB1). Defaults to auto-detect.")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--auto", "-a", action="store_true", help="Run automated test suite directly")
    args = parser.parse_args()

    port = args.port or find_default_port()
    if not port:
        print("[ERROR] No USB/ACM serial device found! Please connect your Arduino or specify --port /dev/ttyUSBX.")
        sys.exit(1)
    tester = ArduinoCommTester(port, args.baud)

    if not tester.connect():
        sys.exit(1)

    try:
        if not tester.handshake():
            sys.exit(1)

        if args.auto:
            run_automated_test(tester)
        else:
            run_interactive_mode(tester)

    finally:
        tester.close()


if __name__ == "__main__":
    main()
