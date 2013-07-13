#ifndef INIT_H
#define INIT_H

const int pwrLEDpin = 0;	// 0 Pi pin 17
const int faultLEDpin = 1;  // 1 Pi pin 18
							// 2 Pi pin 27
							// 3 Pi pin 22
							// 4 Pi pin 23
							// 5 Pi pin 24
							// 6 Pi pin 25
							// 7 Pi pin 4

const int rideHeightOffset = -32; // -32mm means body at 100mm off ground

/* Servo IDs */
const int RM_TIBIA_ID   = 18;
const int RF_COXA_ID    = 2;
const int LR_TIBIA_ID   = 11;
const int LF_FEMUR_ID   = 3;
const int RF_TIBIA_ID   = 6;
const int RM_FEMUR_ID   = 16;
const int RM_COXA_ID    = 14;
const int RR_COXA_ID    = 8;
const int LF_TIBIA_ID   = 5;
const int LF_COXA_ID    = 1;
const int LR_FEMUR_ID   = 9;
const int RR_FEMUR_ID   = 10;
const int LM_TIBIA_ID   = 17;
const int RF_FEMUR_ID   = 4;
const int LM_FEMUR_ID   = 15;
const int RR_TIBIA_ID   = 12;
const int LM_COXA_ID    = 13;
const int LR_COXA_ID    = 7;

const int RIGHT_FRONT   = 0;
const int RIGHT_MIDDLE  = 1;
const int RIGHT_REAR    = 2;
const int LEFT_REAR     = 3;
const int LEFT_MIDDLE   = 4;
const int LEFT_FRONT    = 5;


/* Body Dimensions */
const int X_COXA        = 120;    // MM between front and back legs /2
const int Y_COXA_FB     = 60;     // MM between front/back legs /2
const int Y_COXA_M      = 100;    // MM between two middle legs /2
const int COXA_ANGLE    = 45;     // Angle of coxa from straight forward (deg)

/* Legs */
const int LENGTH_COXA   = 52;     // MM distance from coxa servo to femur servo
const int LENGTH_FEMUR  = 66;     // MM distance from femur servo to tibia servo
const int LENGTH_TIBIA  = 132;    // MM distance from tibia servo to foot

const unsigned int SERVO_UPDATE_PERIOD = 20; //ms

/* Parameters for Commander input */
struct commanderStruct{
  int   Xspeed;
  int   Yspeed;
  int   Rspeed;
  int   bodyTransX;
  int   bodyTransY;
  int   bodyTransZ;
  float bodyRotX;
  float bodyRotY;
  float bodyRotZ;
};
extern commanderStruct commanderInput;

/* Leg parts */
struct footPosStruct{
  int x;
  int y;
  int z;
};

struct footPosCalcStruct{
  long x;
  long y;
  long z;
};

struct jointAnglesStruct{
  float coxa;
  float femur;
  float tibia;
};

struct servoPosStruct{
  int coxa;
  int femur;
  int tibia;
};

struct initialFootPosStruct{
  int x;
  int y;
  int z;
};

struct legBasePosStruct{
  int x;
  int y;
  int z;
};

struct legStruct{
  footPosStruct         footPos;
  footPosCalcStruct     footPosCalc;
  jointAnglesStruct     jointAngles;
  servoPosStruct        servoPos;
  initialFootPosStruct  initialFootPos;
  legBasePosStruct      legBasePos;
  float                 bodyRotZ;
};
extern legStruct        leg[6]; 


/* Body parts */
struct bodyStruct{
  float rotX;
  float rotY;
  float rotZ;
  int   posX;
  int   posY;
  int   posZ;
};
//extern bodyStruct body;
  


void readCommandInputs();
void runIK();
void driveServos();
void legAngleCorrections();
void tripodGait();
void tripodGaitSine();
void rippleGait();
float radians(float deg);
float degrees(float rad);


#endif
