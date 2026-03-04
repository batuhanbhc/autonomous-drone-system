# DroneInfo.msg
# Lightweight info/warn/error text from a node

builtin_interfaces/Time stamp

uint8 LEVEL_INFO=0
uint8 LEVEL_WARN=1
uint8 LEVEL_ERROR=2

uint8 level
string text