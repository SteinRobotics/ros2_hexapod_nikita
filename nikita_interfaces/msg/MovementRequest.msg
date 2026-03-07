std_msgs/Header header
# Types:
uint8 NO_REQUEST = 0
uint8 LAYDOWN = 1
uint8 STAND_UP = 2
uint8 WAITING = 3
uint8 MOVE_TRIPOD = 4
uint8 MOVE_RIPPLE = 5
uint8 MOVE_WAVE = 6
uint8 CONTINUOUS_POSE = 7
uint8 WATCH = 8
uint8 LOOK = 9
uint8 DANCE = 10
uint8 HIGH_FIVE = 11
uint8 LEGS_WAVE = 12 
uint8 BODY_ROLL = 13
uint8 BITE = 14
uint8 STOMP = 15
uint8 CLAP = 16
uint8 SINGLE_POSE = 17
uint8 TESTLEGS = 18
# the following types are not used for normal operation 
uint8 NEUTRAL = 101  # set all servos position 0, used for calibration
uint8 CALIBRATE = 102  # set all feets to kiss point ground, used for calibration

# Directions:
uint8 CLOCKWISE = 0
uint8 ANTICLOCKWISE = 1

uint8 type 0
uint8 direction 0
float64 duration_s 1.0   # duration of 1 cycle
string name




