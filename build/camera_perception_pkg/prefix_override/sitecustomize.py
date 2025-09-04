import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/euntaek/ws_mobile/install/camera_perception_pkg'
