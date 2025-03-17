import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/belvin/Desktop/INFORMATIK/DRONE_PROJECT/ARDUPILOT/ardu_ws/src/p2-drone-formation-control-simulator/install/p2-drone-formation-control-simulator'
