#!/usr/bin/env python3
"""
motor_serial_bridge.py
----------------------
MotorPi에서 실행되며, Arduino와 ZeroMQ 사이의 ‘시리얼 브릿지’ 역할을 담당한다.

기능:
1) ZMQ SUB(6003) → MotorPi 명령 수신 (‘w/a/s/d/x’)
2) 해당 문자열을 Arduino로 시리얼 전송
3) Arduino가 보낸 엔코더 데이터 "LF:x RF:y LR:z RR:k" 수신
4) 이를 다시 ZMQ PUB(6002)로 motor_odom_pub.py 에 전송
"""

import zmq
import serial
import time
import glob

BAUD = 115200

# ----------------------------------------------------
# (1) Arduino 포트 자동 탐지
# ----------------------------------------------------
def find_port():
    ports = glob.glob("/dev/ttyACM*")
    if len(ports) == 0:
        print("[Bridge] No ACM port found.")
        return None
    print("[Bridge] Found ports:", ports)
    return ports[0]

def connect_arduino():
    while True:
        port = find_port()
        if port is None:
            time.sleep(1)
            continue

        try:
            s = serial.Serial(port, BAUD, timeout=0.1)
            print(f"[Bridge] Connected to Arduino on {port}")
            return s
        except Exception as e:
            print("[Bridge] Connection failed:", e)
            time.sleep(1)

# 시리얼 연결
ser = connect_arduino()

# ----------------------------------------------------
# (2) ZMQ 설정
# ----------------------------------------------------
ctx = zmq.Context()

# MotorPi → Arduino 명령 수신 SUB
cmd_sub = ctx.socket(zmq.SUB)
cmd_sub.connect("tcp://127.0.0.1:6003")
cmd_sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("[Bridge] CMD SUB → 6003")

# Arduino → MotorPi 오돔용 엔코더 PUB
enc_pub = ctx.socket(zmq.PUB)
enc_pub.bind("tcp://*:6002")
print("[Bridge] ENC PUB → 6002")

# ZMQ Poller 설정 (비동기)
poller = zmq.Poller()
poller.register(cmd_sub, zmq.POLLIN)

# ----------------------------------------------------
# (3) 메인 루프
# ----------------------------------------------------
while True:

    # ========== 1) ZMQ: MotorPi command 수신 ==========
    socks = dict(poller.poll(timeout=5))
    if cmd_sub in socks:
        cmd = cmd_sub.recv_string()
        print("[Bridge] CMD:", cmd)

        # Arduino로 명령 전송
        try:
            ser.write((cmd + "\n").encode())
        except Exception as e:
            print("[Bridge] Serial write error:", e)
            ser = connect_arduino()

    # ========== 2) Arduino → 엔코더 읽기 ==========
    try:
        line = ser.readline().decode(errors="ignore").strip()

        # 엔코더 데이터 포맷 예:
        # "LF:123 RF:120 LR:122 RR:119"
        if line.startswith("LF:") and "RF:" in line and "LR:" in line and "RR:" in line:
            enc_pub.send_string(line)   # motor_odom_pub.py 로 전송
            # print("[Bridge] ENC:", line)

    except Exception as e:
        print("[Bridge] Serial read error:", e)
        ser = connect_arduino()
