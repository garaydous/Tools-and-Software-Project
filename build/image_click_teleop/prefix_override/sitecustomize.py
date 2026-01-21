import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/maria/ros2_project_ws/src/Tools-and-Software-Project/install/image_click_teleop'
