ltm/EntityMetadata meta

# static fields
string name
uint8 age_bottom
uint8 age_top
uint8 age_avg
uint8 live_phase
uint8 genre
sensor_msgs/Image face

# dynamic fields
string emotion
time last_seen
time last_interacted

uint8 UNKNOWN=0

# Available GENRES
uint8 MASCULINE=1
uint8 FEMININE=2

# Available LIVE_PHASES
uint8 CHILD=1
uint8 ADULT=2