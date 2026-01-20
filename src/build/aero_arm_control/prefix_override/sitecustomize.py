import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ali/Desktop/Ros2_Control/src/install/aero_arm_control'
