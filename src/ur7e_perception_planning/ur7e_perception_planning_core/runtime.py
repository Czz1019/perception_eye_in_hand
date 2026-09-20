"""One GPU owner at a time in combined mode; ROS I/O and CPU guard stay live."""
from threading import RLock

GPU_LOCK = RLock()
