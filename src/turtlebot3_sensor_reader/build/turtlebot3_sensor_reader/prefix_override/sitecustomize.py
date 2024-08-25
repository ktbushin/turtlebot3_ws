import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kosuke/turtlebot3_ws/src/turtlebot3_sensor_reader/install/turtlebot3_sensor_reader'
