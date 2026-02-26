std_msgs/Header header      # current timestamp
uint8 id                   # vehicle id

bool armed                  # armed state (armed, disarmed)
bool ARMED = true
bool DISARM = false

uint8 mode                 # mode (takeoff, land, loiter, orbit)
uint8 MODE_TAKEOFF = 0
uint8 MODE_ORBIT = 1
uint8 MODE_LAND = 2
uint8 MODE_LOITER = 3
uint8 MODE_UNKNOWN = 255