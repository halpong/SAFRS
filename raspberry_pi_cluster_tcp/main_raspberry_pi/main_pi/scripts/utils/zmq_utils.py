#!/usr/bin/env python3
"""
SAFRS Main Pi - Utility Module
Module: zmq_utils

Description:
    ZeroMQ convenience functions for creating publishers
    and subscribers with SAFRS standard settings.
"""

import zmq
import time


def create_subscriber(ip: str, port: int, timeout_ms: int = 5):
    """
    Create ZeroMQ SUB socket.
    - Connects to tcp://{ip}:{port}
    - SUBSCRIBE to all topics
    - Optional receive timeout in ms
    """
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.connect(f"tcp://{ip}:{port}")
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.setsockopt(zmq.RCVTIMEO, timeout_ms)
    return sub


def create_publisher(port: int, startup_delay: float = 0.5):
    """
    Create ZeroMQ PUB socket.
    - Binds to tcp://*:{port}
    - Optional delay to allow subscribers to connect
    """
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://*:{port}")
    time.sleep(startup_delay)
    return pub
