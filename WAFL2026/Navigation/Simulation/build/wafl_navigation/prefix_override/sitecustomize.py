import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/martin/gp/repo26/WAFL2026/Navigation/Simulation/install/wafl_navigation'
