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
#define MOTION_SET_SINUNOID_F_STANDINGINIT 32

#define MOTION_DISPLAY_SAM_CONTROL 11
#define MOTION_DISPLAY_SAM_FEEDBACK 12
#define MOTION_DISPLAY_TURNOFF 13

#define MOTION_ENABLE_CONTROL_BODY_PITCH 17
#define MOTION_DISABLE_CONTROL_BODY_PITCH 18

#define MOTION_ENABLE_CONTROL_BODY_ROLL 19
#define MOTION_DISABLE_CONTROL_BODY_ROLL 20

#define MOTION_ENALBLE_CONTROL_ANKLE_LEFT_ROLL 22
#define MOTION_DISABLE_CONTROL_ANKLE_LEFT_ROLL 23

#define MOTION_ENALBLE_CONTROL_ANKLE_RIGHT_ROLL 24
#define MOTION_DISABLE_CONTROL_ANKLE_RIGHT_ROLL 25

#define MOTION_ENALBLE_CONTROL_ANKLE_LEFT_PITCH 26
#define MOTION_DISABLE_CONTROL_ANKLE_LEFT_PITCH 27

#define MOTION_ENALBLE_CONTROL_ANKLE_RIGHT_PITCH 28
#define MOTION_DISABLE_CONTROL_ANKLE_RIGHT_PITCH 29

#define MOTION_ENALBLE_CONTROL_FOOT_PLACE 30
#define MOTION_DISABLE_CONTROL_FOOT_PLACE 31


//#define MOTION_ENALBLE_CONTROL_FOOT_PLACE 26
//#define MOTION_DISABLE_CONTROL_FOOT_PLACE 27

//#define MOTION_ENALBLE_CONTROL_FOOT_PLACE 26
//#define MOTION_DISABLE_CONTROL_FOOT_PLACE 27

//#define MOTION_ENALBLE_CONTROL_FOOT_PLACE 26
//#define MOTION_DISABLE_CONTROL_FOOT_PLACE 27
struct Sys_flag_struct{
    unsigned char sinunoid_motion:1;
    unsigned char enable_sam:1;
    unsigned char feedback_sensor_available:1;
    unsigned char feedback_tfCal_available:1;
    unsigned char feedback_sam_available:1;
    unsigned char controller_body_pitch:1;// base on IMU
    unsigned char controller_body_roll:1;// base on joint angle feedback

    unsigned char controller_ankle_left_roll:1;
    unsigned char controller_ankle_left_pitch:1;
    unsigned char controller_ankle_right_roll:1;
    unsigned char controller_ankle_right_pitch:1;

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
    unsigned char controller_impedance:1;
    unsigned char controller_joint[12];

    unsigned char wait_for_stable;

}sys_flag;
unsigned int pos12LowerLink[25];
/*
 * =========== motion variable ===========
 */

//====================hardware============
//const int samPos12_hardware[25]={1627,1700,2050,2052,663,3446,1260,2910,2751,2190,1260,2370,
//                                 2025,2050,2050,2050,2050,2050,3100,940,0,0,2170,1500,2050};// zero of the real system 12bits
const int samPos12_hardware[25]={1620,1680,2050,2060,663,3325,1260,2910,2730,2173,1237,2370,
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
double pose_sitdown[25]={5,-5,-54,54,154,-154,132,-132,-3,3,7,-7,
                         0,0,0,0,0,0,0,0,0,0,0,0,0};
const unsigned int sitdown_averageTorq[12]={3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};
const unsigned char sitdown_samP[12]={40,40,40,40,40,40,40,40,40,40,40,40};
const unsigned char sitdown_samD[12]={10,10,10,10,10,10,10,10,10,10,10,10};
//================================ init standing for walking| from standing================
// int pose_init_walking[25]={0,0,-200,200,300,-300,200,-200,0,0,20,-20,
//                                 0,0,0,0,0,0,0,0,0,0,0,0,0};
double pose_init_walking[25]={0,0,-19,19,25,-25,15,-15,0,0,0,0,
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

double pose_standing_1[25]={5,-5,-54,54,154,-154,132,-132,-3,3,7,-7,
                            0,0,0,0,0,0,0,0,0,0,0,0,0};
double pose_standing_2[25]={2,-2,-45,45,120,-120,130,-130,0,0,2,-2,
                            0,0,0,0,0,0,0,0,0,0,0,0,0};
double pose_standing_3[25]={0,0,-5,5,-2,2,-5,5,0,0,0,0,
                            0,0,0,0,0,0,0,0,0,0,0,0,0};
const unsigned int standing_averageTorq[12]={3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};
const unsigned char standing_samP[12]={40,40,40,40,40,40,40,40,40,40,40,40};
const unsigned char standing_samD[12]={10,10,10,10,10,10,10,10,10,10,10,10};
const unsigned int numOfFrames_standing[2]={300,400};


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


double pose_step_comLeft[25]={6,6,-17,17,25,-25,17,-17,-6,-6,1.5,-1.5,
                              0,0,0,0,0,0,0,0,0,0,0,0,0};

double pose_step_init[25]={0,0,-17,17,25,-25,17,-17,0,0,1.5,-1.5,
                           0,0,0,0,0,0,0,0,0,0,0,0,0};

double pose_step_rightUp[25]={6,6,-37,17,65,-25,38,-17,-6,-6,1.5,-1.5,
                              0,0,0,0,0,0,0,0,0,0,0,0,0};

// double pose_step_rightDown[25]={6,6,-19,17,25,-25,22,-17,-6,-6,1.5,-1.5,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};
double pose_step_rightDown[25]={0,0,-19,17,25,-25,22,-17,0,0,1.5,-1.5,
                                0,0,0,0,0,0,0,0,0,0,0,0,0};

double pose_step_impedance_1[25]={0,0,-22,22,35,-35,22,-22,0,0,1.5,-1.5,
                                  0,0,0,0,0,0,0,0,0,0,0,0,0};



double pose_step_comRight[25]={-8,-8,-17,17,25,-25,17,-17,8,8,1.5,-1.5,
                               0,0,0,0,0,0,0,0,0,0,0,0,0};

double pose_step_leftUp[25]={-8,-8,-17,37,25,-65,17,-38,8,8,1.5,-1.5,
                             0,0,0,0,0,0,0,0,0,0,0,0,0};

// double pose_step_leftDown[25]={-8,-8,-17,17,25,-25,17,-17,8,8,1.5,-1.5,
//                           0,0,0,0,0,0,0,0,0,0,0,0,0};
double pose_step_leftDown[25]={0,0,-17,17,25,-25,17,-17,0,0,1.5,-1.5,
                               0,0,0,0,0,0,0,0,0,0,0,0,0};

double pose_step_impedance_2[25]={0,0,-22,22,35,-35,22,-22,0,0,1.5,-1.5,
                                  0,0,0,0,0,0,0,0,0,0,0,0,0};

double pose_step_7[25]={0,0,-20,20,33,-33,20,-20,0,0,1.5,-1.5,
                        0,0,0,0,0,0,0,0,0,0,0,0,0};

const unsigned int steping_averageTorq[12]={3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};
//{3500,3500,3500,3500,3500,3500,3500,3500,3500,3500,3500,3500};
const unsigned char steping_samP[12]={40,40,40,40,40,40,40,40,40,40,40,40};
//{45,45,45,45,45,45,45,45,45,45,45,45};
const unsigned char steping_samD[12]={10,10,10,10,10,10,10,10,10,10,10,10};
//{30,30,30,30,30,30,30,30,30,30,30,30};
//{10,10,10,10,10,10,10,10,10,10,10,10};
//{30,30,30,30,30,30,30,30,30,30,30,30};
//{35,35,35,35,35,35,35,35,35,35,35,35};
const unsigned int numOfFrames_steping[2]={20,25};

unsigned char taskStepCurrentScene=0;



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
#define TASK_STEP_SINUNOID 10
#define TASK_STEP_COMMAND 11
//#define TASK_POSES_TEST_  6
//#define TASK_NEW_WALKING_ 7

#define TASK_IDLE_   12
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
    //    double slope_begin[25];
    //    double slope_end[25];
    //    double slope_midle[25];
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
    double amp_filtered;
    double pre_amp;
    double delta_amp;
} leftZMP,rightZMP;

double delta_amp;
unsigned char stable_pose_flag=0;
unsigned int stable_cycle=0;
struct MyIMU{
    double roll;
    double pitch;
    double yaw;
    double roll_rate;
    double pitch_rate;
    double yaw_rate;
}imu;

double COMpos=0;
double leftFootPos[3];
double rightFootPos[3];
double footsDistance=0;
double COMposSetpoint=0;
unsigned int currentSamPos12[25];
double currentSAMPosDegree[25];
unsigned char currentSamAvail[25];
unsigned int currentControlledSamPos12[25];

//==================== PID ====================================

#define BODY_PITCH_SETPOINT_STANDING    0//-15
#define BODY_ROLL_SETPOINT_STANDING     0

#define SAMPLING_TIME_ 0.01
double controller_output[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


myPID pid_impedance_right;
myPID pid_impedance_left;
double pid_impedace_ouputIntegrate_right=0;
double pid_impedace_ouputIntegrate_left=0;
myPID pid_joint[12];


myPID pid_body_pitch;
const double body_roll_setpoint=0;
const double body_pitch_setpoint=-5;

myPID pid_ankle_left_roll;
myPID pid_ankle_right_roll;

myPID pid_ankle_left_pitch;
myPID pid_ankle_right_pitch;
const double ankle_left_roll_setpoint=0;
const double ankle_right_pitch_setpoint=0;
const double ankle_left_pitch_setpoint=0;
const double ankle_right_roll_setpoint=0;

myPID pid_foot_place;
const double foot_place_setpoint=0.15;

//====================== sinunoid step task=========================
double pose_step_sinunoid_init[25]={0,0,-20,20,33,-33,20,-20,0,0,1.5,-1.5,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0};

#define STEP_WAIT_FOR_STABLE_1     1
#define BODY_TILT_OFFSET 7//10
#define ANKLE_PITCH_OFFSET 0
double ankle_pitch_offset=ANKLE_PITCH_OFFSET;
#define BODY_HEIGHT_LEFT 0.48//0.48
#define BODY_HEIGHT_RIGHT 0.478//0.475
#define FOOT_POS_X_OFFSET 0.07//0.07
double foot_pos_x_offset=FOOT_POS_X_OFFSET;
#define FOOT_POS_Y_OFFSET_LEFT  0.02//0.02
#define FOOT_POS_Y_OFFSET_RIGHT 0.02
#define FOOT_POS_Z_AMP 2//0.8//2
#define FOOT_POS_Z_OFFSET 1.85//0.65//1.85

double foot_pos_z_amp=FOOT_POS_Z_AMP;
double foot_pos_z_offset=FOOT_POS_Z_OFFSET;
#define ARRAY_SIZE(w,h) (w*h)
#define INDEX(x,y) ((x-1)+4*(y-1))
#define STEP_WIDTH 0.03//0.03
#define STEP_HEIGHT 0.03//0.03


const double gravity=9.81;
const double ZCOM=0.7;
const double omega_n=sqrt(gravity/ZCOM);
const double L6=0.08;
const double L3=0.21;
const double L4=0.209;

//#define FOOT_POS_Y_OFFSET 0.02
double foot_right_posY_offset=FOOT_POS_Y_OFFSET_RIGHT;//0.03;
double foot_left_posY_offset=FOOT_POS_Y_OFFSET_LEFT;//0.035;
double AcomX=STEP_HEIGHT;//0.025;//0.07;
double AcomY=STEP_WIDTH;//0.03;
double body_tilt_offset =BODY_TILT_OFFSET;
const double freq=0.6;//0.6;
const double omega=2*M_PI*freq;
const double periodT=1/freq;
double step_duration=periodT/2;

double AzmpX=(1+pow(omega/omega_n,2))*AcomX;
double AzmpY=(1+pow(omega/omega_n,2))*AcomY;

double phiX_left=-M_PI/2;
double phiY_left=0;
double phiZ_left=0;
double phiX_right=M_PI/2;
double phiY_right=0;
double phiZ_right=M_PI;

double foot_left_position[3];
double foot_left_mat[ARRAY_SIZE(4,4)]={1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1};
double foot_right_position[3];
double foot_right_mat[ARRAY_SIZE(4,4)]={1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1};
double calAngle[6];


double step_timer=0;

//========= damper process=====
#define FOOT_POS_Z_DAMPING 0.01//0.02
#define STEP_STANCE 0
#define STEP_UP 1
#define STEP_DOWN 2

double right_leg_height;
double left_leg_height;
double damping_left;
double damping_right;
unsigned char flag_trigger_damping_left=0;
unsigned char flag_enable_damping_left=0;
unsigned char flag_trigger_damping_right=0;
unsigned char flag_enable_damping_right=0;
double step_damping_timer_t0=0;
double step_damping_left_timer_t=0;
double step_damping_right_timer_t=0;
double omega_damping=11.3;//11.3
unsigned char step_left_status=STEP_STANCE;
unsigned char step_right_status=STEP_STANCE;
//========= x motion process======
unsigned char flag_step_firstTime=0;
unsigned char flag_enable_step_in=0;
unsigned char flag_enable_step_out=0;

unsigned char flag_step_in_left=0;
unsigned char flag_step_in_right=0;

unsigned char flag_step_out_left=0;
unsigned char flag_step_out_right=0;

unsigned char flag_step_up_left=0;
unsigned char flag_step_up_right=0;
double step_Xmotion_timer_t0=0;
double step_Xmotion_timer=0;
unsigned char flag_Xmotion_resetTimer_left=0;
unsigned char flag_Xmotion_resetTimer_right=0;

double omega_X=omega*M_PI/(M_PI-2*asin(foot_pos_z_offset/foot_pos_z_amp))*2.5;
//double omega_X=50;
double periodT_X=2*M_PI/omega_X;

// =========yaw motion===========
unsigned char flag_enable_step_yaw=0;
double Ayaw_right=0;
double Ayaw_left=0;
double phiYaw=0;
double omega_yaw=omega_X/2;
double foot_left_yaw=0;
double foot_right_yaw=0;
const  double foot_left_roll=1/180.0*M_PI;
double c_roll=cos(foot_left_roll);
double s_roll=sin(foot_left_roll);

unsigned char rotate_step_count=0;
void inverseKinematic(double *PMatrix,double Px,double Py, double Pz,double *angle){
    double L05=sqrt(pow(Px+L6*(*(PMatrix+INDEX(1,3))),2)+pow(Py+L6*(*(PMatrix+INDEX(2,3))),2)+pow(Pz+L6*(*(PMatrix+INDEX(3,3))),2));
    *(angle+3)=acos((pow(L05,2)-pow(L3,2)-pow(L4,2))/2/L3/L4);
    double P_inverse_x=-(Px*(*(PMatrix+INDEX(1,1)))+Py*(*(PMatrix+INDEX(2,1)))+Pz*(*(PMatrix+INDEX(3,1))));
    double P_inverse_y=-(Px*(*(PMatrix+INDEX(1,2)))+Py*(*(PMatrix+INDEX(2,2)))+Pz*(*(PMatrix+INDEX(3,2))));
    double P_inverse_z=-(Px*(*(PMatrix+INDEX(1,3)))+Py*(*(PMatrix+INDEX(2,3)))+Pz*(*(PMatrix+INDEX(3,3))));
    double L06yz=sqrt(pow(P_inverse_y,2)+pow(P_inverse_z,2));
    double L05yz=sqrt(pow(P_inverse_y,2)+pow(P_inverse_z-L6,2));

    *(angle+5)=copysign(acos((pow(L06yz,2)-pow(L05yz,2)-pow(L6,2))/2/L05yz/L6),P_inverse_y);
    *(angle+4)=-asin(P_inverse_x/L05)-acos(-(pow(L3,2)-pow(L05,2)-pow(L4,2))/2/L4/L05);

    *(angle+1)=asin((*(PMatrix+INDEX(3,2)))*cos(*(angle+5))-(*(PMatrix+INDEX(3,3)))*sin(*(angle+5)));
    *angle=atan2(((*(PMatrix+INDEX(1,2)))*cos(*(angle+5))-(*(PMatrix+INDEX(1,3)))*sin(*(angle+5)))/-cos(*(angle+1)),((*(PMatrix+INDEX(2,2)))*cos(*(angle+5))-(*(PMatrix+INDEX(2,3)))*sin(*(angle+5)))/cos(*(angle+1)));
    *(angle+2)=acos((-(*(PMatrix+INDEX(3,3)))*cos(*(angle+4))*cos(*(angle+5))-(*(PMatrix+INDEX(3,2)))*cos(*(angle+4))*sin(*(angle+5))+(*(PMatrix+INDEX(3,1)))*sin(*(angle+4)))/-cos(*(angle+1)))-(*(angle+3));
}

//==== leg id======================
#define STEP_LEGID_LEFT           0
#define STEP_LEGID_RIGHT          1


//==== mode ===========
#define STEP_MODE_INPLACE         1
#define STEP_MODE_FORWARD_IN      2
#define STEP_MODE_FORWARD_OUT     3
#define STEP_MODE_FORWARD_NORMAL  7
#define STEP_MODE_BACKWARD_NORMAL 8
#define STEP_MODE_BACKWARD_IN     4
#define STEP_MODE_BACKWARD_OUT    5
#define STEP_MODE_ROTATE          6




#define STEP_CMD_WALK_FORWARD_LEFT     0
#define STEP_CMD_WALK_FORWARD_RIGHT    1

#define STEP_CMD_WALK_ROTATE_LEFT      2
#define STEP_CMD_WALK_ROTATE_RIGHT     3

#define STEP_CMD_WALK_BACKWARD_LEFT    4
#define STEP_CMD_WALK_BACKWARD_RIGHT   5

#define STEP_CMD_WALK_SIDE_LEFT        6
#define STEP_CMD_WALK_SIDE_RIGHT       7

#define STEP_CMD_WALK_INPLACE       8


unsigned char step_command=0;
unsigned char step_number=0;
unsigned char step_count=0;
unsigned char leg_current_id=0;




void setUp_stepMotion(unsigned char mode,unsigned char legID,double rotation_degree){
    double yaw=rotation_degree/180*M_PI;
    step_duration=periodT/2;
    flag_enable_step_in=0;
    flag_enable_step_out=0;
    body_tilt_offset=BODY_TILT_OFFSET;

    Ayaw_left=0;
    Ayaw_right=0;
    foot_left_posY_offset=FOOT_POS_Y_OFFSET_LEFT;
    foot_right_posY_offset=FOOT_POS_Y_OFFSET_RIGHT;
    flag_enable_step_yaw=0;

    foot_pos_z_amp=FOOT_POS_Z_AMP;
    foot_pos_z_offset=FOOT_POS_Z_OFFSET;


    foot_pos_x_offset=FOOT_POS_X_OFFSET;

    ankle_pitch_offset=ANKLE_PITCH_OFFSET;
    AcomY=STEP_WIDTH;
    switch (mode) {
    case STEP_MODE_INPLACE:
        AcomX=0;
        break;
    case STEP_MODE_FORWARD_IN:
        AcomX=STEP_HEIGHT;
        flag_enable_step_in=1;
        break;
    case STEP_MODE_FORWARD_OUT:
        AcomX=STEP_HEIGHT;
        flag_enable_step_out=1;
        break;
    case STEP_MODE_BACKWARD_IN:
        AcomX=-STEP_HEIGHT;
        //        foot_pos_x_offset=0.02;
//        ankle_pitch_offset=3;
//        AcomY=0.04;
        //         body_tilt_offset=25;
        flag_enable_step_in=1;
        foot_pos_z_amp=2.6;
        foot_pos_z_offset=2.45;
//        omega_X=omega*M_PI/(M_PI-2*asin(foot_pos_z_offset/foot_pos_z_amp))*1.5;
//        periodT_X=2*M_PI/omega_X;
        break;
    case STEP_MODE_BACKWARD_OUT:
        AcomX=-STEP_HEIGHT;
        //         foot_pos_x_offset=0.02;
//        ankle_pitch_offset=3;
//        AcomY=0.04;
        //         body_tilt_offset=25;
        flag_enable_step_out=1;
        foot_pos_z_amp=2.6;
        foot_pos_z_offset=2.45;
//        omega_X=omega*M_PI/(M_PI-2*asin(foot_pos_z_offset/foot_pos_z_amp))*1.5;
//        periodT_X=2*M_PI/omega_X;
        break;
    case STEP_MODE_BACKWARD_NORMAL:
        AcomX=-STEP_HEIGHT;
        //         foot_pos_x_offset=0.02;
//        ankle_pitch_offset=3;
//        AcomY=0.04;
        //         body_tilt_offset=25;
        foot_pos_z_amp=2.6;
        foot_pos_z_offset=2.45;
//        omega_X=omega*M_PI/(M_PI-2*asin(foot_pos_z_offset/foot_pos_z_amp))*1.5;
//        periodT_X=2*M_PI/omega_X;
        break;
    case STEP_MODE_FORWARD_NORMAL:
        AcomX=STEP_HEIGHT;
        break;

    case STEP_MODE_ROTATE:

        flag_enable_step_yaw=1;
        rotate_step_count=0;
        AcomX=0;
//         AcomX=-0.005;
        //        body_tilt_offset=2;
        step_duration=periodT;
        if(legID==STEP_LEGID_LEFT){
            Ayaw_left=yaw;
        }else if(legID==STEP_LEGID_RIGHT){
            Ayaw_right=yaw;
        }
        break;
    default:
        break;
    }

    omega_X=omega*M_PI/(M_PI-2*asin(foot_pos_z_offset/foot_pos_z_amp));
    periodT_X=2*M_PI/omega_X;
    omega_yaw=omega_X/2;


    if(legID==STEP_LEGID_LEFT){
        phiX_left=-M_PI/2;
        phiY_left=0;
        phiZ_left=0;
        phiX_right=M_PI/2;
        phiY_right=0;
        phiZ_right=M_PI;

    }
    else if(legID==STEP_LEGID_RIGHT){
        phiX_left=M_PI/2;
        phiY_left=M_PI;
        phiZ_left=M_PI;
        phiX_right=3*M_PI/2;
        phiY_right=M_PI;
        phiZ_right=0;
    }

    step_timer=0;
    sys_flag.sinunoid_motion=1;
    currentScene.flag.enable=1;
}

#endif // MAINSENSORHUB_H
