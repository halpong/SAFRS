#!/usr/bin/env python3
"""
motor_command_sub.py
---------------------
Main Pi에서 생성되는 /cmd_vel → JSON 명령을
Motor Control RPi에서 수신하여 Arduino로 전달하기 위한 브릿지.

동작:
1) Main Pi에서 ZMQ PUB(5000)로 lx, az(선속도/각속도) JSON 전송
2) MotorPi에서 SUB 후 cmd 해석
3) lx, az 값을 기반으로 ‘w/a/s/d/x’ 단일 문자 생성
4) motor_serial_bridge.py 로 전달 (ZMQ PUB → 5003)
5) motor_serial_bridge.py 가 Arduino에 시리얼로 전송
"""

#!/usr/bin/env python3
import zmq
import json

MAIN_PI_IP = "172.30.1.78"
ZMQ_MAIN_CMD_PORT = 5000     # Main Pi publishes cmd_vel JSON

ctx = zmq.Context()

# Main Pi → SUB
main_sub = ctx.socket(zmq.SUB)
main_sub.connect(f"tcp://{MAIN_PI_IP}:{ZMQ_MAIN_CMD_PORT}")
main_sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("[MotorPi] main CMD SUB ready (5000)")

# Bridge → Arduino PUB
bridge_pub = ctx.socket(zmq.PUB)
bridge_pub.bind("tcp://*:5003")
print("[MotorPi] bridge CMD PUB bind (5003)")

while True:
    raw = main_sub.recv_string()
    print("[MotorPi] Received:", raw)

    try:
        data = json.loads(raw)
        lx = data["lx"]
        az = data["az"]
    except:
        continue

    if lx > 0.1:
        cmd = "w"
    elif lx < -0.1:
        cmd = "s"
    elif az > 0.1:
        cmd = "a"
    elif az < -0.1:
        cmd = "d"
    else:
        cmd = "x"

    print("[MotorPi] SEND TO BRIDGE:", cmd)
    bridge_pub.send_string(cmd)

