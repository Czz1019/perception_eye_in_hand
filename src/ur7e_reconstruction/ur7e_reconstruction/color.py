"""Decode ROS color images without assuming depth alignment or tightly packed rows."""
import numpy as np


def rgb_image(msg, stride=1):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding.lower())
    if channels is None or msg.step < msg.width * channels or len(msg.data) < msg.step * msg.height:
        raise ValueError('Unsupported or malformed RGB image')
    data = np.ndarray((msg.height, msg.width, channels), np.uint8,
                      buffer=msg.data, strides=(msg.step, channels, 1))[::stride, ::stride, :3]
    if msg.encoding.lower().startswith('bgr'):
        data = data[..., ::-1]
    return np.ascontiguousarray(data)
