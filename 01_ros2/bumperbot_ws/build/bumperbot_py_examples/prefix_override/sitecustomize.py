import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sdn/ntp/Robotics-Odometry_Control/01_ros2/bumperbot_ws/install/bumperbot_py_examples'
