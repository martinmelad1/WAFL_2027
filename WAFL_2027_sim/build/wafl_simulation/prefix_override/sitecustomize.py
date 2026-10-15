import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mireille-youssef/WAFL_2027/install/wafl_simulation'
