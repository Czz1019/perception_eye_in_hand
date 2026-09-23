import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/czz/perception_eye_in_hand/install/realtime_nvblox'
