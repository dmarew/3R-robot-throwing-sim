import math
M_PI = math.pi
WIDTH = 1160
HEIGHT = 380

# button geometry
BOX = 18
BOXW = WIDTH//4 - 4

# sub-panel geometry and placement
CENTER_X = 120
CENTER_Y = 200
SCALE = 60.0

# world to pixmap panels (drawing) transformations
W2DR = lambda num: int(SCALE*1000*num)//1000
W2DX = lambda num:	 CENTER_X + int(SCALE*1000*num)//1000
W2DY = lambda num:  CENTER_Y - int(SCALE*1000.0*num)//1000

# pixmap to world */

D2WR = lambda num: float(num)/SCALE
D2WX = lambda num: float(num-CENTER_X)/SCALE
D2WY = lambda num: float(CENTER_Y-num)/SCALE
FREEFALL =0
PD_CONTROL= 1
THROW=  2
TD_CONTROL = 3
TORQUE_MAX=20.0
KP_ARM = 9000.0
KD_ARM = 200.0
TIMEOUT = 60     #   /* seconds worth of simulation       */
DT =  0.0005   #/* the time increment between frames */
RENDER_RATE = 20        #/* render every twentieth state      */
SERVO_RATE = 1        #/* servo rate at 1000Hz (1 msec)     */
TIMER_UPDATE = 5

GRAVITY = 9.8
#Robot
NARMS = 1
NLINKS = 3
NJOINTS = 3
NFRAMES = 5
R_JOINT = 0.03  # the radius of a joint */
L1 = 0.2020  # the length of link 1 */
L2 = 0.2020  # the length of link 2 */
L3 = 0.1136  # the length of link 3 */
M1 = 1.0
M2 = 1.0
M3 = 0.8
THETA1_HOME = 0.0
THETA1_REF = M_PI/2.0
THETA2_HOME = 0.0
THETA2_REF = 5.0*M_PI/8.0
THETA3_HOME = 0.0
THETA3_REF = M_PI/2.0

RELEASE_POINT = (M_PI/4.0)

GROUND_PLANE_Y = -(L1+L2+L3)
# CONSTANTS AND STRUCTURES THAT DEFINE MECHANISMS FOR THE SIMULATOR   */
NOTYPE = -1        # KINEMATIC SPECIFICATIONS       */
NOAXIS = -1
REVOLUTE = 0
PRISMATIC = 1
XAXIS = 0
YAXIS = 1
ZAXIS = 2
#BALL
R_OBJ=0.0508 # m - regulation baseball */
M_OBJECT= 0.1421*2.0# kg - regulation baseball */
X = 0
Y = 1
XDOT = 2
YDOT = 3

SGN = lambda x: 1.0 if x > 0 else -1.0
SQR = lambda x: x*x
MIN = lambda x, y: x if x < y else y
MAX = lambda x, y: x if x > y else y
