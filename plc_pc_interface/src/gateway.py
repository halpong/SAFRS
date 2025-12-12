#!/usr/bin/env python3
"""
SAFRS PC–PLC–STM32–RPi Gateway
- Reads PLC state over RS232
- Sends command to STM32 when trigger detected
- Forwards START signal to Raspberry Pi via UDP
"""

import serial
import socket
import time
import yaml
import os
from typing import Optional


# ==============================================================
# 1. YAML CONFIG LOAD
# ==============================================================

def load_yaml(path: str) -> dict:
    """Load a YAML configuration file."""
    if not os.path.exists(path):
        raise FileNotFoundError(f"[ERROR] Config file not found: {path}")

    with open(path, "r") as f:
        return yaml.safe_load(f)


CONFIG = load_yaml("config/connection.yaml")
PROTOCOL = load_yaml("config/protocol.yaml")

serial_cfg = CONFIG["serial"]
udp_cfg = CONFIG["udp"]
parse_cfg = PROTOCOL["parse_rules"]
timing_cfg = PROTOCOL["timing"]

print("[INFO] Loaded configuration files.")


# ==============================================================
# 2. SERIAL & UDP SETUP
# ==============================================================

def open_serial(port: str, baud: int, timeout: float):
    """Open a serial port with given parameters."""
    try:
        return serial.Serial(port, baud, timeout=timeout)
    except Exception as e:
        raise RuntimeError(f"[ERROR] Cannot open serial port {port}: {e}")


plc = open_serial(
    serial_cfg["plc_port"],
    serial_cfg["plc_baudrate"],
    serial_cfg["timeout_pc_plc_ms"] / 1000.0
)

stm32 = open_serial(
    serial_cfg["stm32_port"],
    serial_cfg["stm32_baudrate"],
    serial_cfg["timeout_pc_stm32_ms"] / 1000.0
)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("[INFO] PC–PLC–STM32–RPi Link initialized.")


# ==============================================================
# 3. PROTOCOL FRAME BUILDER
# ==============================================================

def build_enq_frame() -> bytes:
    """Build ENQ frame defined in protocol.yaml."""
    hex_str = PROTOCOL["plc_enq"]["enq_frame_hex"]
    return bytes.fromhex(hex_str)


ENQ_FRAME = build_enq_frame()
FRAME_END = bytes.fromhex(parse_cfg["frame_end_hex"])
REQUIRED_DIGITS = parse_cfg["numeric_digits"]
ON_VALUE = parse_cfg["on_value"]


# ==============================================================
# 4. PLC RESPONSE PARSER
# ==============================================================

def parse_plc_response(frame: bytes) -> str:
    """Convert PLC frame to ASCII string."""
    try:
        ascii_str = frame.decode(errors="ignore")
    except Exception:
        ascii_str = str(frame)

    print(f"[PLC RX ASCII] {ascii_str}")
    return ascii_str


def extract_numeric_value(ascii_frame: str) -> Optional[str]:
    """Extract last N digits from PLC ASCII response."""
    numeric_only = "".join(filter(str.isdigit, ascii_frame))
    if len(numeric_only) < REQUIRED_DIGITS:
        return None
    return numeric_only[-REQUIRED_DIGITS:]


# ==============================================================
# 5. MAIN LOOP
# ==============================================================

def main():
    print("[INFO] Starting ENQ polling loop...")
    buffer = b""

    while True:

        # ------------------------------
        # 1) Send ENQ to PLC
        # ------------------------------
        plc.write(ENQ_FRAME)
        print(f"[PC → PLC] ENQ sent ({ENQ_FRAME.hex()})")
        time.sleep(0.02)

        # ------------------------------
        # 2) Read PLC response
        # ------------------------------
        data = plc.read(100)

        if data:
            buffer += data

            # Frame completed when ETX (0x03) received
            if FRAME_END in buffer:
                frame = buffer
                buffer = b""  # reset buffer

                ascii_frame = parse_plc_response(frame)
                last_value = extract_numeric_value(ascii_frame)

                if last_value:
                    print(f"[PARSE] Last {REQUIRED_DIGITS} digits = {last_value}")

                    # ------------------------------
                    # TRIGGER: M0 == ON
                    # ------------------------------
                    if last_value == ON_VALUE:
                        print(f"[INFO] Trigger detected (M0=ON). Waiting {timing_cfg['delay_after_detect_sec']} seconds...")

                        time.sleep(timing_cfg["delay_after_detect_sec"])

                        # 1) Send command to STM32
                        stm32.write(b"S")
                        print("[PC → STM32] Sent 'S'")

                        # 2) Notify Raspberry Pi via UDP
                        sock.sendto(b"START", (udp_cfg["rpi_ip"], udp_cfg["rpi_port"]))
                        print(f"[PC → RPi] Sent 'START' → {udp_cfg['rpi_ip']}:{udp_cfg['rpi_port']}")

                        print("[INFO] Task completed. Exiting.")
                        break

        time.sleep(0.1)

    print("[INFO] Program finished.")


# ==============================================================
# ENTRY POINT
# ==============================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")
    except Exception as e:
        print(f"[ERROR] {e}")
