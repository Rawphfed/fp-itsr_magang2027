import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rawfed/Programming/simulasi-2027_ws/install/robocon_2027_webots'
