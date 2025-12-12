#!/usr/bin/env python3
"""
SAFRS Main Pi - Utility Module
Module: image_utils

Description:
    Helper functions for image encoding/decoding
    used in ZMQ camera transport pipeline.
"""

import base64
import numpy as np
import cv2


def decode_base64_image(b64_string: str):
    """
    Decode Base64 string → OpenCV BGR image.
    Returns:
        frame (np.ndarray) or None if decode fails.
    """
    try:
        jpg = base64.b64decode(b64_string)
        arr = np.frombuffer(jpg, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return frame
    except Exception:
        return None
