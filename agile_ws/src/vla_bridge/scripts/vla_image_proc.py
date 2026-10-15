#!/usr/bin/env python3
"""JPEG decode / letterbox / encode shared by image_node and bag_to_uavflow."""

import cv2
import numpy as np


def decode_jpeg_to_rgb(data):
    buf = np.frombuffer(bytes(bytearray(data)), dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def letterbox_rgb(rgb, size):
    height, width = rgb.shape[:2]
    scale = float(size) / float(max(height, width))
    new_w = max(1, min(size, int(round(width * scale))))
    new_h = max(1, min(size, int(round(height * scale))))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(rgb, (new_w, new_h), interpolation=interp)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    top = (size - new_h) // 2
    left = (size - new_w) // 2
    canvas[top : top + new_h, left : left + new_w] = resized
    return canvas


def center_crop_rgb(rgb, size):
    height, width = rgb.shape[:2]
    side = min(height, width)
    y0 = (height - side) // 2
    x0 = (width - side) // 2
    crop = rgb[y0 : y0 + side, x0 : x0 + side]
    interp = cv2.INTER_AREA if side > size else cv2.INTER_LINEAR
    return cv2.resize(crop, (size, size), interpolation=interp)


def resize_rgb(rgb, size, mode):
    if mode == "center_crop":
        return center_crop_rgb(rgb, size)
    return letterbox_rgb(rgb, size)


def encode_jpeg_rgb(rgb, quality):
    # OpenCV imencode expects BGR; JPEG bytes are then RGB-correct for PIL / Image.open.
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    return buf.tobytes()
