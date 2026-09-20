import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wqj/realtime_nvblox_ws/install/ur7e_nvblox'
