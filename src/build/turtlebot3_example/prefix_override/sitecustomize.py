import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kosuke/turtlebot3_ws/src/install/turtlebot3_example'
