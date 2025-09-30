std_msgs/Header header
uint8 NO_REQUEST = 0
uint8 LAYDOWN = 1
uint8 STAND_UP = 2
uint8 WAITING = 3
uint8 MOVE = 4
uint8 MOVE_TO_STAND = 5
uint8 WATCH = 6
uint8 LOOK_LEFT = 7
uint8 LOOK_RIGHT = 8
uint8 DANCE = 9
uint8 HIGH_FIVE = 10
uint8 LEGS_WAVE = 11 
uint8 BODY_ROLL = 12
uint8 BITE = 13
uint8 STOMP = 14
uint8 CLAP = 15
uint8 TRANSPORT = 16 # minimal ground space, used for transport
uint8 TESTBODY = 17
uint8 TESTLEGS = 18
# the following types are not used for normal operation 
uint8 NEUTRAL = 101  # set all servos position 0, used for calibration
uint8 CALIBRATE = 102  # set all feets to kiss point ground, used for calibration

uint8 type 0
uint32 duration_ms 1000
string name

#from cmd_vel: velocity are used mainly for type MOVE
geometry_msgs/Twist velocity #linear and angular
Pose body # position and orientation of the body



