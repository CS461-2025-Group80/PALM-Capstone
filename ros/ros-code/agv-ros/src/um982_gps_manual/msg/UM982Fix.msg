# UM982 GPS combined message for logging
std_msgs/Header header

# Position
float64 latitude       # degrees
float64 longitude      # degrees
float64 altitude       # meters above sea level

# Heading (from $GNHPR)
float32 heading        # degrees true north (0-360)
float32 pitch          # degrees
float32 roll           # degrees

# Velocity (from $GNRMC)
float32 speed          # m/s ground speed
float32 course         # degrees course over ground

# Fix quality (from $GNGGA)
# 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
int8 fix_quality
int8 satellites
float32 hdop
