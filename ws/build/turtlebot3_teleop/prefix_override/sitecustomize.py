import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sentient-beings/mnt/ws/install/turtlebot3_teleop'
