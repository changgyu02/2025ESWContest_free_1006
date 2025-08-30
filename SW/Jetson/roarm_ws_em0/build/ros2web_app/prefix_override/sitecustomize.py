import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/changgyu/roarm_ws/roarm_ws_em0/install/ros2web_app'
