#ifndef MAINSENSORHUB_H
#define MAINSENSORHUB_H

#define NUM_OF_SAM_ 25

#define SENSOR_TURNOFF 0
#define SENSOR_SEND_FILTERED_50HZ 1
#define SENSOR_SEND_FILTERED_125HZ 2
#define SENSOR_SEND_RAW_50HZ 3
#define SENSOR_SEND_RAW_125HZ 4


#define SAM_FB_TURNOFF 0
#define SAM_FB_LOWERLINK_POS12_50HZ 1
#define SAM_FB_LOWERLINK_POS12_125HZ 2
#define SAM_FB_FULLBODY_POS12_50HZ 3
#define SAM_FB_FULLBODY_POS12_125HZ 4

#define DISPLAY_TURN_OFF_ 0
#define DISPLAY_SAM_CONTROL_ 1
#define DISPLAY_SAM_FEEDBACK_ 2
/*
 * =========== timer variable ===========
 */
#define LOOP_RATE_1000Hz_ 1000

/*
 * COUNT is defined for timer 1ms
 */
#define COUNT_50_HZ_	20
#define COUNT_100_HZ_	10
#define COUNT_125_HZ_   8
struct TimerCountType{
    unsigned char Hz_100;
    unsigned char Hz_50;
    unsigned char Hz_125;
}Timer_Count;

struct FlagTimerType{
    unsigned char Hz_50:1;
    unsigned char Hz_100:1;
    unsigned char Hz_125:1;
}FlagTimer;
//======timer handler run in 1000Hz=========
void Timer_handler(){
    Timer_Count.Hz_50++;
    Timer_Count.Hz_100++;
    Timer_Count.Hz_125++;
    if(Timer_Count.Hz_50==COUNT_50_HZ_){
        Timer_Count.Hz_50=0;
        FlagTimer.Hz_50=1;
    }
    if(Timer_Count.Hz_100==COUNT_100_HZ_)
    {
        Timer_Count.Hz_100=0;
        FlagTimer.Hz_100=1;
    }
    if(Timer_Count.Hz_125==COUNT_125_HZ_)
    {
        Timer_Count.Hz_125=0;
        FlagTimer.Hz_125=1;
    }
}

#define MOTION_DISABLE_FB_SENSOR 1
#define MOTION_ENBALE_FB_SENSOR 2

#define MOTION_DISABLE_FB_SAM 3
#define MOTION_ENABLE_FB_SAM 4

#define MOTION_ENABLE_FB_ALL 5
#define MOTION_DISABLE_FB_ALL 0

#define MOTION_ENABLE_SAM 6
#define MOTION_DISABLE_SAM 7

#define MOTION_STANDING_F_SITDOWN 8
#define MOTION_INIT_F_IDLE 9
#define MOTION_SITDOWN_F_INIT 10
#define MOTION_STANDINGINIT_F_STANDING 14
#define MOTION_STANDING_F_STADINGINIT 15
#define MOTION_SITDOWN_F_STANDING 16
#define MOTION_STEP_F_STANDINGINIT 21

#define MOTION_DISPLAY_SAM_CONTROL 11
#define MOTION_DISPLAY_SAM_FEEDBACK 12
#define MOTION_DISPLAY_TURNOFF 13

#define MOTION_ENABLE_CONTROL_BODY_PITCH 17
#define MOTION_DISABLE_CONTROL_BODY_PITCH 18

#define MOTION_ENABLE_CONTROL_BODY_ROLL 19
#define MOTION_DISABLE_CONTROL_BODY_ROLL 20

#define MOTION_ENALBLE_CONTROL_ANKLE_LEFT 22
#define MOTION_DISABLE_CONTROL_ANKLE_LEFT 23

#define MOTION_ENALBLE_CONTROL_ANKLE_RIGHT 24
#define MOTION_DISABLE_CONTROL_ANKLE_RIGHT 25

#define MOTION_ENALBLE_CONTROL_FOOT_PLACE 26
#define MOTION_DISABLE_CONTROL_FOOT_PLACE 27

#define MOTION_ENALBLE_CONTROL_JOINT_8 28
#define MOTION_DISABLE_CONTROL_JOINT_8 29

//#define MOTION_ENALBLE_CONTROL_FOOT_PLACE 26
//#define MOTION_DISABLE_CONTROL_FOOT_PLACE 27

//#define MOTION_ENALBLE_CONTROL_FOOT_PLACE 26
//#define MOTION_DISABLE_CONTROL_FOOT_PLACE 27

//#define MOTION_ENALBLE_CONTROL_FOOT_PLACE 26
//#define MOTION_DISABLE_CONTROL_FOOT_PLACE 27
struct Sys_flag_struct{
    unsigned char enable_sam:1;
    unsigned char feedback_sensor_available:1;
    unsigned char feedback_tfCal_available:1;
    unsigned char feedback_sam_available:1;
    unsigned char controller_body_pitch:1;// base on IMU
    unsigned char controller_body_roll:1;// base on joint angle feedback

    unsigned char controller_ankle_left:1;
    unsigned char controller_ankle_right:1;
    unsigned char controller_foot_place:1;

    unsigned char controller_joint_0:1;
    unsigned char controller_joint_1:1;
    unsigned char controller_joint_8:1;
    unsigned char controller_joint_9:1;

    unsigned char controller_joint_3:1;
    unsigned char controller_joint_5:1;
    unsigned char controller_joint_7:1;

    unsigned char controller_joint_2:1;
    unsigned char controller_joint_4:1;
    unsigned char controller_joint_6:1;
    unsigned char controller_joint[12];
}sys_flag;
unsigned int pos12LowerLink[25];
/*
 * =========== motion variable ===========
 */

//====================hardware============
const int samPos12_hardware[25]={1640,1689,2050,2052,663,3446,1260,2910,2761,2163,1260,2370,
                               2025,2050,2050,2050,2050,2050,3100,940,0,0,2170,1500,2050};// zero of the real system 12bits

const double pos12bitTorad=0.083*M_PI/180;
const double pos12bitTodegree=0.083;
const double degreeToPose12=1/0.083;

double pos12_double[25];


double delta_pos12[25];

double posSetPointDegree[25];
double delta_posDegree[25];
/*
 * ==================== Pose ================
 */
 unsigned char default_samP[12]={30,30,30,30,30,30,30,30,30,30,30,30};
 unsigned char default_samI[12]={0,0,0,0,0,0,0,0,0,0,0,0};
 unsigned char default_samD[12]={5,5,5,5,5,5,5,5,5,5,5,5};
 unsigned int default_averageTorq[12]={2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000};

//================================== sit down| from init posef===============================
// int pose_sitdown[25]={0,0,-652,652,1852,-1852,1604,-1604,-39,39,91,-39,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};
 double pose_sitdown[25]={0,0,-54,54,154,-154,132,-132,-3,3,7,-7,
                          0,0,0,0,0,0,0,0,0,0,0,0,0};
const unsigned int sitdown_averageTorq[12]={3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};
 const unsigned char sitdown_samP[12]={40,40,40,40,40,40,40,40,40,40,40,40};
 const unsigned char sitdown_samD[12]={15,15,15,15,15,15,15,15,15,15,15,15};
//================================ init standing for walking| from standing================
// int pose_init_walking[25]={0,0,-200,200,300,-300,200,-200,0,0,20,-20,
//                                 0,0,0,0,0,0,0,0,0,0,0,0,0};
 double pose_init_walking[25]={0,0,-16,16,25,-25,16,-16,0,0,2,-2,
                                 0,0,0,0,0,0,0,0,0,0,0,0,0};
//{0,0,-300,300,522,-522,235,-235,-13,13,26,-26,
//                                 0,0,0,0,0,0,0,0,0,0,0,0,0};
//================================ standing| from sitdown================
//int pose_standing_1[25]={0,0,-652,652,1852,-1852,1604,-1604,-39,39,91,-39,
//                            0,0,0,0,0,0,0,0,0,0,0,0,0};
// int pose_standing_2[25]={0,0,-522,522,1408,-1408,1343,-1343,0,0,26,-26,
//                               0,0,0,0,0,0,0,0,0,0,0,0,0};
// int pose_standing_3[25]={0,0,-65,65,-26,26,-13,13,0,0,20,-20,
//                               0,0,0,0,0,0,0,0,0,0,0,0,0};

 double pose_standing_1[25]={0,0,-54,54,154,-154,132,-132,-3,3,7,-7,
                          0,0,0,0,0,0,0,0,0,0,0,0,0};
 double pose_standing_2[25]={0,0,-43,43,117,-117,111,-111,0,0,2,-2,
                            0,0,0,0,0,0,0,0,0,0,0,0,0};
 double pose_standing_3[25]={0,0,-5,5,0,0,-1,1,0,0,2,-2,
                               0,0,0,0,0,0,0,0,0,0,0,0,0};
const unsigned int standing_averageTorq[12]={3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};
const unsigned char standing_samP[12]={40,40,40,40,40,40,40,40,40,40,40,40};
const unsigned char standing_samD[12]={30,30,30,30,30,30,30,30,30,30,30,30};
const unsigned int numOfFrames_standing[2]={200,300};


//================================ steping test| from initwaking================
// int pose_step_1[25]={0,0,-200,200,300,-300,200,-200,0,0,20,-20,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};
// int pose_step_2[25]={0,0,-400,200,700,-300,400,-200,0,0,20,-20,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};

// int pose_step_3[25]={0,0,-200,200,300,-300,200,-200,0,0,20,-20,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};


// int pose_step_4[25]={0,0,-200,200,300,-300,200,-200,0,0,20,-20,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};

// int pose_step_5[25]={0,0,-200,400,300,-700,200,-400,0,0,20,-20,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};

// int pose_step_6[25]={0,0,-200,200,300,-300,200,-200,0,0,20,-20,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};

// int pose_step_7[25]={0,0,-250,250,400,-400,250,-250,0,0,20,-20,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};


 double pose_step_0[25]={6,6,-17,17,25,-25,17,-17,-6,-6,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

 double pose_step_1[25]={0,0,-17,17,25,-25,17,-17,0,0,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

 double pose_step_2[25]={6,6,-33,17,58,-25,33,-17,-6,6,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

 double pose_step_3[25]={6,6,-17,17,25,-25,17,-17,-6,-6,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};


 double pose_step_4[25]={0,0,-17,17,25,-25,17,-17,0,0,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

 double pose_step_8[25]={-8,-8,-17,17,25,-25,17,-17,8,8,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

 double pose_step_5[25]={-8,-8,-17,33,25,-58,17,-33,8,8,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

 double pose_step_6[25]={-8,-8,-17,17,25,-25,17,-17,8,8,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

 double pose_step_7[25]={0,0,-20,20,33,-33,20,-20,0,0,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

const unsigned int steping_averageTorq[12]={3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};
const unsigned char steping_samP[12]={40,40,40,40,40,40,40,40,40,40,40,40};
const unsigned char steping_samD[12]={30,30,30,30,30,30,30,30,30,30,30,30};
const unsigned int numOfFrames_steping[2]={30,30};





/*
 * ================== task variable =====================
 */

#define TASK_INIT_F_IDLE 0
//#define TASK_TEST_ 1
#define TASK_SITDOWN_F_INIT  2
#define TASK_STANDING_F_SITDOWN  3
#define TASK_STANDINGINIT_F_STANDING  4
#define TASK_STANDING_F_STANDINGINIT 5
#define TASK_SITDOWN_F_STANDING 6
#define TASK_STEP_TEST 7
#define TASK_COM_TRANSFER  8
#define TASK_STANDINGINIT_F_STEP  9
//#define TASK_POSES_TEST_  6
//#define TASK_NEW_WALKING_ 7

#define TASK_IDLE_   10
typedef struct{
    unsigned char startFlag:1;
    unsigned char finishFlag:1;
    unsigned char id;
    unsigned char scene;
    unsigned char preId;
    unsigned char setup_para;
    unsigned char setup_para_count;
}task_type;
task_type task;


/*
 * ============================== Scene =========================
 */
class Scene_class{
    unsigned int samTorq[25];
    unsigned char samP[25];
    unsigned char samI[25];
    unsigned char samD[25];
    double slope_begin[25];
    double slope_end[25];
    double slope_midle[25];
public:
//    unsigned int *beginPose=NULL;//dagerous thing,no infor about memory
//    unsigned int *endPose=NULL;
//    unsigned int buf_beginPose[NUM_OF_SAM_];
//    unsigned int buf_endPose[NUM_OF_SAM_];

    double *beginPose=NULL;//dagerous thing,no infor about memory
    double *endPose=NULL;

    double buf_beginPose[NUM_OF_SAM_];
    double buf_endPose[NUM_OF_SAM_];

    unsigned int frame;
    unsigned int numOfFrame;
    struct Flag{
        unsigned char start:1;
        unsigned char finish:1;
        unsigned char enable:1;
        unsigned char delay:1;
        unsigned char controller_stable:1;
    }flag;

//    void setBeginPose(unsigned int *value);
//    void setEndPose(unsigned int *value);
//    void setUpMyScene(unsigned int fr, int *beginpose, int *endpose);

    void setFrame(unsigned int value);
    void setNumOfFrame(unsigned int value);
    void setDelayScene(unsigned int fr);
    void setUpMyScene(unsigned int fr, double *beginpose, double *endpose);
    void setBeginPose(double *value);
    void setEndPose(double *value);

    void mapSoftToHardPose(int *softwarePose, unsigned int *hardwarePose, unsigned char size);
     void mapHardToSoftPose(unsigned int *hardwarePose, int *softwarePose, unsigned char size);
//     void mapDegreeToPos12(double *degreePose,unsigned int *hardwarePose,unsigned char size);
     void mapPos12ToDegree(unsigned int *hardwarePose,double *degreePose,unsigned char size);



    Scene_class(){
    }
};
Scene_class currentScene;

/*
 * ====================sensor variable============================
 *
 */
struct MyZMP{
    unsigned int rawForceSensor[4];
    unsigned int filterForceSensor[4];
    double posX;
    double posY;
    double amp;
} leftZMP,rightZMP;

struct MyIMU{
    double roll;
    double pitch;
    double yaw;
}imu;

double leftFootPos[3];
double rightFootPos[3];
double footsDistance=0;
double COMposSetpoint=0;
unsigned int currentSamPos12[25];
double currentSAMPosDegree[25];
unsigned char currentSamAvail[25];
unsigned int currentControlledSamPos12[25];

//==================== PID ====================================
#define SAMPLING_TIME_ 0.01
double controller_output[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
myPID pid_body_roll_0_1;
myPID pid_body_roll_8_9;

myPID pid_joint_0;
myPID pid_joint_1;
myPID pid_joint_8;
myPID pid_joint_9;

myPID pid_joint_3;
myPID pid_joint_5;
myPID pid_joint_7;

myPID pid_joint_2;
myPID pid_joint_4;
myPID pid_joint_6;

myPID pid_joint[12];


myPID pid_body_pitch;
const double body_roll_setpoint=0;
const double body_pitch_setpoint=-5;

myPID pid_ankle_left;
myPID pid_ankle_right;
const double ankle_left_COP_setpoint=0;
const double ankle_right_COP_setpoint=0;

myPID pid_foot_place;
const double foot_place_setpoint=0.15;

#endif // MAINSENSORHUB_H
