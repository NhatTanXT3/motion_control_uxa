#include <iostream>
#include <ros/ros.h>
using namespace std;
#include "uxa_motion_control/mypid.h"
#include "uxa_motion_control/main_uxa_motion_control.h"

#include "uxa_motion_control/motionCmdMsg.h"
#include "uxa_motion_control/dataSensorMsg.h"
#include "uxa_motion_control/cmdSensorMsg.h"

#include "uxa_motion_control/SAMcmdMsg.h"
#include "uxa_motion_control/SAMJointPos12Msg.h"
#include "uxa_motion_control/SAMJointStateMsg.h"
#include "uxa_motion_control/SAMJointPIDMsg.h"
#include "uxa_motion_control/SAMJointTorqueMsg.h"
#include "uxa_motion_control/SAMtfCalMsg.h"

#include "uxa_motion_control/displayCmdMsg.h"
#include "uxa_motion_control/uxaParameter_displayMsg.h"
#include "uxa_motion_control/uxaSetPIDMsg.h"




//#include ""
#define SAMPLING_TIME_ 0.01
double controller_output[25]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
myPID pid_body_roll_0_1;
myPID pid_body_roll_8_9;

myPID pid_body_pitch;
const double body_roll_setpoint=0;
const double body_pitch_setpoint=-5;

myPID pid_ankle_left;
myPID pid_ankle_right;
const double ankle_left_COP_setpoint=0;
const double ankle_right_COP_setpoint=0;

myPID pid_foot_place;
const double foot_place_setpoint=0.14;

void PID_control_init(){
    //    pid_body_roll_0_1 =new myPID();
    //    pid_body_pitch = new myPID();
    pid_body_roll_0_1.sampling_time=SAMPLING_TIME_;
    pid_body_roll_0_1.reset_parameters();
    pid_body_roll_0_1.KP=10;
    pid_body_roll_0_1.KI=40;
    pid_body_roll_0_1.KD=0;
    pid_body_roll_0_1.I_limit=100;
    pid_body_roll_0_1.D_limit=50;
    pid_body_roll_0_1.output_limit=200;
    pid_body_roll_0_1.set_point=body_roll_setpoint;

    pid_body_roll_8_9.sampling_time=SAMPLING_TIME_;
    pid_body_roll_8_9.reset_parameters();
    pid_body_roll_8_9.KP=10;
    pid_body_roll_8_9.KI=40;
    pid_body_roll_8_9.KD=0;
    pid_body_roll_8_9.I_limit=100;
    pid_body_roll_8_9.D_limit=50;
    pid_body_roll_8_9.output_limit=200;
    pid_body_roll_8_9.set_point=body_roll_setpoint;

    pid_body_pitch.sampling_time=SAMPLING_TIME_;
    pid_body_pitch.reset_parameters();
    pid_body_pitch.KP=1;
    pid_body_pitch.KI=30;//70;
    pid_body_pitch.KD=0;
    pid_body_pitch.I_limit=100;
    pid_body_pitch.D_limit=50;
    pid_body_pitch.output_limit=200;
    pid_body_pitch.set_point=body_pitch_setpoint;

    pid_ankle_left.sampling_time=SAMPLING_TIME_;
    pid_ankle_left.reset_parameters();
    pid_ankle_left.KP=0;//10;
    pid_ankle_left.KI=10;//5;
    pid_ankle_left.KD=0;//0.5;
    pid_ankle_left.I_limit=100;
    pid_ankle_left.D_limit=50;
    pid_ankle_left.output_limit=200;
    pid_ankle_left.set_point=ankle_left_COP_setpoint;

    pid_ankle_right.sampling_time=SAMPLING_TIME_;
    pid_ankle_right.reset_parameters();
    pid_ankle_right.KP=0;
    pid_ankle_right.KI=0;
    pid_ankle_right.KD=0;
    pid_ankle_right.I_limit=5;
    pid_ankle_right.D_limit=5;
    pid_ankle_right.output_limit=10;
    pid_ankle_right.set_point=ankle_right_COP_setpoint;

    pid_foot_place.sampling_time=SAMPLING_TIME_;
    pid_foot_place.reset_parameters();
    pid_foot_place.KP=0;
    pid_foot_place.KI=10000;
    pid_foot_place.KD=0;
    pid_foot_place.I_limit=70;
    pid_foot_place.D_limit=50;
    pid_foot_place.output_limit=100;
    pid_foot_place.set_point=foot_place_setpoint;
}
void PID_reset()
{
    pid_body_roll_0_1.reset_parameters();
    pid_body_pitch.reset_parameters();
    pid_ankle_left.reset_parameters();
    pid_ankle_right.reset_parameters();
    pid_foot_place.reset_parameters();

    for(unsigned char i=0;i<NUM_OF_SAM_;i++)
    {
        controller_output[i]=0;
    }
    //    memset(controller_output,'\0',25);
}

ros::Publisher sensor_cmd_pub;
ros::Publisher sam_cmd_pub;
ros::Publisher display_cmd_pub;

#define PID_SET_BODY_PITCH_ 1
#define SETPOINT_SET_BODY_PITCH_ 2
#define PID_SET_BODY_ROLL_1_2_ 3
#define SETPOINT_SET_BODY_ROLL_1_2_ 4

#define PID_ANKLE_LEFT_ 5
#define SETPOINT_ANKLE_LEFT_ 6

#define PID_ANKLE_RIGHT_ 7
#define SETPOINT_ANKLE_RIGHT_ 8

#define COM_POS_SETPOINT_ 9

#define PID_FOOT_PLACE_ 10
#define SETPOINT_FOOT_PLACE_ 11

#define PID_SET_BODY_ROLL_8_9_ 12
#define SETPOINT_SET_BODY_ROLL_8_9_ 13

void sub_function_setPID(const uxa_motion_control::uxaSetPIDMsg::ConstPtr& msg){
    switch(msg->pidtype){
    case PID_SET_BODY_PITCH_:
        pid_body_pitch.KP=msg->Pvalue;
        pid_body_pitch.KI=msg->Ivalue;
        pid_body_pitch.KD=msg->Dvalue;
        ROS_INFO("PID_SET_BODY_PITCH_ KP=%f| KI=%f| KD=%f",pid_body_pitch.KP,pid_body_pitch.KI,pid_body_pitch.KD);
        break;
    case SETPOINT_SET_BODY_PITCH_:
        pid_body_pitch.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_SET_BODY_PITCH_ setpoint=%f",pid_body_pitch.set_point);
        break;
    case PID_SET_BODY_ROLL_1_2_:
        pid_body_roll_0_1.KP=msg->Pvalue;
        pid_body_roll_0_1.KI=msg->Ivalue;
        pid_body_roll_0_1.KD=msg->Dvalue;
        ROS_INFO("PID_SET_BODY_ROLL_ KP=%f| KI=%f| KD=%f",pid_body_roll_0_1.KP,pid_body_roll_0_1.KI,pid_body_roll_0_1.KD);
        break;
    case SETPOINT_SET_BODY_ROLL_1_2_:
        pid_body_roll_0_1.set_point=msg->setPoint;
        pid_body_roll_8_9.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_SET_BODY_ROLL_ setpoint=%f",pid_body_roll_0_1.set_point);
        break;

    case PID_SET_BODY_ROLL_8_9_:
        pid_body_roll_8_9.KP=msg->Pvalue;
        pid_body_roll_8_9.KI=msg->Ivalue;
        pid_body_roll_8_9.KD=msg->Dvalue;
        ROS_INFO("PID_SET_BODY_ROLL_ KP=%f| KI=%f| KD=%f",pid_body_roll_8_9.KP,pid_body_roll_8_9.KI,pid_body_roll_8_9.KD);
        break;
    case SETPOINT_SET_BODY_ROLL_8_9_:
        pid_body_roll_8_9.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_SET_BODY_ROLL_ setpoint=%f",pid_body_roll_8_9.set_point);
        break;
    case PID_ANKLE_LEFT_:
        pid_ankle_left.KP=msg->Pvalue;
        pid_ankle_left.KI=msg->Ivalue;
        pid_ankle_left.KD=msg->Dvalue;
        ROS_INFO("PID_ANKLE_LEFT_ KP=%f| KI=%f| KD=%f",pid_ankle_left.KP,pid_ankle_left.KI,pid_ankle_left.KD);
        break;
    case SETPOINT_ANKLE_LEFT_:
        pid_ankle_left.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_ANKLE_LEFT_ setpoint=%f",pid_ankle_left.set_point);
        break;
    case COM_POS_SETPOINT_:
        pid_ankle_left.I_limit=msg->I_limit;
        //        constrain(COMposSetpoint,-100,100);
        ROS_INFO(" pid_ankle_left.I_limit=%f", pid_ankle_left.I_limit);

    case PID_ANKLE_RIGHT_:
        pid_ankle_right.KP=msg->Pvalue;
        pid_ankle_right.KI=msg->Ivalue;
        pid_ankle_right.KD=msg->Dvalue;
        ROS_INFO("PID_ANKLE_RIGHT_ KP=%f| KI=%f| KD=%f",pid_ankle_right.KP,pid_ankle_right.KI,pid_ankle_right.KD);
        break;

    case SETPOINT_ANKLE_RIGHT_:
        pid_ankle_right.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_ANKLE_RIGHT_ setpoint=%f",pid_ankle_right.set_point);
        break;

    case PID_FOOT_PLACE_:
        pid_foot_place.KP=msg->Pvalue;
        pid_foot_place.KI=msg->Ivalue;
        pid_foot_place.KD=msg->Dvalue;
        ROS_INFO("PID_ANKLE_LEFT_ KP=%f| KI=%f| KD=%f",pid_foot_place.KP,pid_foot_place.KI,pid_foot_place.KD);
        break;
    case SETPOINT_FOOT_PLACE_:
        pid_foot_place.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_ANKLE_LEFT_ setpoint=%f",pid_foot_place.set_point);
        break;
    default:
        ROS_INFO("PID_SET_BODY_PITCH_ KP=%f| KI=%f| KD=%f",pid_body_pitch.KP,pid_body_pitch.KI,pid_body_pitch.KD);
        ROS_INFO("SETPOINT_SET_BODY_PITCH_ setpoint=%f",pid_body_pitch.set_point);

        ROS_INFO("PID_SET_BODY_ROLL_ KP=%f| KI=%f| KD=%f",pid_body_roll_0_1.KP,pid_body_roll_0_1.KI,pid_body_roll_0_1.KD);
        ROS_INFO("SETPOINT_SET_BODY_ROLL_ setpoint=%f",pid_body_roll_0_1.set_point);

        ROS_INFO("PID_ANKLE_LEFT_ KP=%f| KI=%f| KD=%f",pid_ankle_left.KP,pid_ankle_left.KI,pid_ankle_left.KD);
        ROS_INFO("SETPOINT_ANKLE_LEFT_ setpoint=%f",pid_ankle_left.set_point);
        ROS_INFO("PID_ANKLE_RIGHT_ KP=%f| KI=%f| KD=%f",pid_ankle_right.KP,pid_ankle_right.KI,pid_ankle_right.KD);
        ROS_INFO("SETPOINT_ANKLE_RIGHT_ setpoint=%f",pid_ankle_right.set_point);

        ROS_INFO("PID_ANKLE_LEFT_ KP=%f| KI=%f| KD=%f",pid_foot_place.KP,pid_foot_place.KI,pid_foot_place.KD);
        ROS_INFO("SETPOINT_ANKLE_LEFT_ setpoint=%f",pid_foot_place.set_point);
        break;
    }
}
void sub_function_cmd(const uxa_motion_control::motionCmdMsg::ConstPtr& msg){

    uxa_motion_control::cmdSensorMsg sensor_command;
    uxa_motion_control::SAMcmdMsg  sam_command;
    uxa_motion_control::displayCmdMsg display_command;

    switch (msg->command){
    case MOTION_DISABLE_FB_SENSOR:
        ROS_INFO("MOTION_DISABLE_FB_SENSOR : %d", msg->command);
        sensor_command.command=SENSOR_TURNOFF;
        sensor_cmd_pub.publish(sensor_command);

        break;
    case MOTION_ENBALE_FB_SENSOR:
        ROS_INFO("MOTION_ENBALE_FB_SENSOR: %d", msg->command);
        sensor_command.command=SENSOR_SEND_FILTERED_125HZ;
        sensor_cmd_pub.publish(sensor_command);
        break;

    case MOTION_DISABLE_FB_SAM:
        ROS_INFO("MOTION_DISABLE_FB_SAM : %d", msg->command);
        sam_command.command=SAM_FB_TURNOFF;
        sam_cmd_pub.publish(sam_command);
        break;
    case MOTION_ENABLE_FB_SAM:
        ROS_INFO("MOTION_ENABLE_FB_SAM: %d", msg->command);
        sam_command.command=SAM_FB_LOWERLINK_POS12_125HZ;
        sam_cmd_pub.publish(sam_command);
        break;
    case MOTION_ENABLE_FB_ALL:
        sam_command.command=SAM_FB_LOWERLINK_POS12_125HZ;
        sam_cmd_pub.publish(sam_command);
        sensor_command.command=SENSOR_SEND_FILTERED_125HZ;
        sensor_cmd_pub.publish(sensor_command);
        break;
    case MOTION_ENABLE_SAM:
        ROS_INFO("enable sam : %d", msg->command);
        sys_flag.enable_sam=1;
        break;
    case MOTION_DISABLE_SAM:
        ROS_INFO("disable sam: %d", msg->command);
        sys_flag.enable_sam=0;
        break;

    case MOTION_STANDING_F_SITDOWN:
        if(task.preId==TASK_SITDOWN_F_INIT|task.preId==TASK_SITDOWN_F_STANDING){
            ROS_INFO("MOTION_STANDING %d", msg->command);
            task.id=TASK_STANDING_F_SITDOWN;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;
    case MOTION_INIT_F_IDLE:
        if(task.preId==TASK_IDLE_){
            ROS_INFO("MOTION_INIT %d", msg->command);
            task.id=TASK_INIT_F_IDLE;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;
    case MOTION_SITDOWN_F_INIT:
        if(task.preId==TASK_INIT_F_IDLE){
            ROS_INFO("MOTION_SITDOWN %d", msg->command);
            task.id=TASK_SITDOWN_F_INIT;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }

        //======================
        //        task.id=TASK_SITDOWN_F_INIT;
        break;
    case MOTION_STANDINGINIT_F_STANDING:
        if(task.preId==TASK_STANDING_F_SITDOWN|task.preId==TASK_STANDING_F_STANDINGINIT){
            ROS_INFO("MOTION_STANDING_INIT %d", msg->command);
            task.id=TASK_STANDINGINIT_F_STANDING;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;
    case MOTION_STANDING_F_STADINGINIT:
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_TEST){
            ROS_INFO("MOTION_STANDING %d", msg->command);
            task.id=TASK_STANDING_F_STANDINGINIT;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;
    case MOTION_SITDOWN_F_STANDING:
        if(task.preId==TASK_STANDING_F_STANDINGINIT|task.preId==TASK_STANDING_F_SITDOWN){
            ROS_INFO("MOTION_SITDOWN %d", msg->command);
            task.id=TASK_SITDOWN_F_STANDING;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;
    case MOTION_STEP_F_STANDINGINIT:
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STEP_TEST|task.preId==TASK_STANDINGINIT_F_STEP){
            ROS_INFO("MOTION_STEP_F_STANDINGINIT %d", msg->command);
            task.id=TASK_STEP_TEST;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;


    case MOTION_DISPLAY_SAM_CONTROL:
        ROS_INFO("DISPLAY_SAM_CONTROL: %d", msg->command);
        display_command.command=DISPLAY_SAM_CONTROL_;
        display_cmd_pub.publish(display_command);
        break;
    case MOTION_DISPLAY_SAM_FEEDBACK:
        ROS_INFO("MOTION_DISPLAY_SAM_FEEDBACK: %d", msg->command);
        display_command.command=DISPLAY_SAM_FEEDBACK_;
        display_cmd_pub.publish(display_command);
        break;
    case MOTION_DISPLAY_TURNOFF:
        ROS_INFO("MOTION_DISPLAY_TURNOFF: %d", msg->command);
        display_command.command=DISPLAY_TURN_OFF_;
        display_cmd_pub.publish(display_command);
        break;
    case MOTION_ENABLE_CONTROL_BODY_PITCH:
        ROS_INFO("MOTION_ENABLE_CONTROL_BODY_PITCH: %d", msg->command);
        sys_flag.controller_body_pitch=1;
        break;
    case MOTION_DISABLE_CONTROL_BODY_PITCH:
        ROS_INFO("MOTION_DISABLE_CONTROL_BODY_PITCH: %d", msg->command);
        sys_flag.controller_body_pitch=0;
        break;

    case MOTION_ENABLE_CONTROL_BODY_ROLL:
        ROS_INFO("MOTION_ENABLE_CONTROL_BODY_ROLL: %d", msg->command);
        sys_flag.controller_body_roll=1;
        break;
    case MOTION_DISABLE_CONTROL_BODY_ROLL:
        ROS_INFO("MOTION_DISABLE_CONTROL_BODY_ROLL: %d", msg->command);
        sys_flag.controller_body_roll=0;
        break;

    case MOTION_ENALBLE_CONTROL_ANKLE_LEFT:
        ROS_INFO("MOTION_ENALBLE_CONTROL_ANKLE_LEFT: %d", msg->command);
        sys_flag.controller_ankle_left=1;
        break;
    case MOTION_DISABLE_CONTROL_ANKLE_LEFT:
        ROS_INFO("MOTION_DISABLE_CONTROL_ANKLE_LEFT: %d", msg->command);
        sys_flag.controller_ankle_left=0;
        break;

    case MOTION_ENALBLE_CONTROL_ANKLE_RIGHT:
        ROS_INFO("MOTION_ENALBLE_CONTROL_ANKLE_RIGHT: %d", msg->command);
        sys_flag.controller_ankle_right=1;
        break;
    case MOTION_DISABLE_CONTROL_ANKLE_RIGHT:
        ROS_INFO("MOTION_DISABLE_CONTROL_ANKLE_RIGHT: %d", msg->command);
        sys_flag.controller_ankle_right=0;
        break;

    case MOTION_ENALBLE_CONTROL_FOOT_PLACE:
        ROS_INFO("MOTION_ENALBLE_CONTROL_FOOT_PLACE: %d", msg->command);
        sys_flag.controller_foot_place=1;
        break;
    case MOTION_DISABLE_CONTROL_FOOT_PLACE:
        ROS_INFO("MOTION_DISABLE_CONTROL_FOOT_PLACE: %d", msg->command);
        sys_flag.controller_foot_place=0;
        break;

    default:
        ROS_INFO("feeback disable all : %d", msg->command);
        sam_command.command=SAM_FB_TURNOFF;
        sam_cmd_pub.publish(sam_command);
        sensor_command.command=SENSOR_TURNOFF;
        sensor_cmd_pub.publish(sensor_command);
        break;
    }
}


void sensor_callback(const uxa_motion_control::dataSensorMsg::ConstPtr& msg){
    //    ROS_INFO("sensor data: %d",msg->zmp_P0 );
    sys_flag.feedback_sensor_available=1;
    rightZMP.filterForceSensor[0]=msg->zmp_P0;
    rightZMP.filterForceSensor[1]=msg->zmp_P1;
    rightZMP.filterForceSensor[2]=msg->zmp_P2;
    rightZMP.filterForceSensor[3]=msg->zmp_P3;
    leftZMP.filterForceSensor[0]=msg->zmp_P4;
    leftZMP.filterForceSensor[1]=msg->zmp_P5;
    leftZMP.filterForceSensor[2]=msg->zmp_P6;
    leftZMP.filterForceSensor[3]=msg->zmp_P7;
    rightZMP.amp=msg->right_amplitude;
    rightZMP.posX=msg->right_x;;
    rightZMP.posY=msg->right_y;

    leftZMP.amp=msg->left_amplitude;
    leftZMP.posX=msg->left_x;
    leftZMP.posY=msg->left_y;
    imu.roll=msg->body_roll;
    imu.pitch=msg->body_pitch;
    imu.yaw=msg->body_yaw;
}

void sam_callback(const uxa_motion_control::SAMJointStateMsg::ConstPtr& msg){
    //    ROS_INFO("sam data: %d",msg->SAMPos12[0]);
    sys_flag.feedback_sam_available=1;
    for(unsigned char i=0;i<NUM_OF_SAM_;i++)
    {
        //        unsigned int dataPos=(Store_chr[i*4+4]&0x7F)+((Store_chr[i*4+3]&0x1F)<<7);
        //        unsigned char index=Store_chr[i*4+2]&0x1F;

        //        if(abs((int)dataPos-(int)pre_samPos12[index])<DELTA_SAM_NOISE){
        //            samPos12[index]=dataPos;
        //            samPos12Avail[index]=1;

        //        }else{
        //            ROS_ERROR("error sam feedback: %d",index);
        //        }

        //        pre_currentSamPos12[index]=msg->SAMPos12[i];
        currentSamPos12[i]=msg->SAMPos12[i];
        currentSamAvail[i]=msg->SAMPos12Avail[i];
    }
}
void sam_tfCal_callback(const uxa_motion_control::SAMtfCalMsg::ConstPtr& msg){
    sys_flag.feedback_tfCal_available=1;
    //    for(unsigned char i=0;i<NUM_OF_SAM_;i++)
    //    {
    //        currentSamPos12[i]=msg->SAMPos12[i];
    //        currentSamAvail[i]=msg->SAMPos12Avail[i];
    //    }
    leftFootPos[0]=msg->CN4;
    leftFootPos[1]=msg->CN6;
    leftFootPos[2]=msg->CN7;

    rightFootPos[0]=msg->CN1;
    rightFootPos[1]=msg->CN2;
    rightFootPos[2]=msg->CN3;
    footsDistance=leftFootPos[0]-rightFootPos[0];
}

uxa_motion_control::SAMJointPos12Msg samPos12;
uxa_motion_control::SAMJointPIDMsg samPID;
uxa_motion_control::SAMJointTorqueMsg samTorque;
ros::Publisher sam_pos12_pub;
ros::Publisher sam_pid_pub;
ros::Publisher sam_torque_pub;

uxa_motion_control::uxaParameter_displayMsg plotMsg;
ros::Publisher plot_pub;
void task_handle();
void controller_handle();
int main(int argc, char **argv){
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE_1000Hz_);

    sensor_cmd_pub =  n.advertise<uxa_motion_control::cmdSensorMsg>("sensor_sub",200);
    ros::Subscriber motion_cmd_sub=n.subscribe<uxa_motion_control::motionCmdMsg>("motion_cmd_sub",200,sub_function_cmd);
    ros::Subscriber setPID_sub =n.subscribe<uxa_motion_control::uxaSetPIDMsg>("setPID",200,sub_function_setPID);

    ros::Subscriber sensor_data_sub =  n.subscribe<uxa_motion_control::dataSensorMsg>("sensor_pub",1000,sensor_callback);
    ros::Subscriber sam_joint_state_sub =  n.subscribe<uxa_motion_control::SAMJointStateMsg>("sam_pub",1000,sam_callback);
    ros::Subscriber sam_tfCal_sub = n.subscribe<uxa_motion_control::SAMtfCalMsg>("sam_tfCal_pub",500,sam_tfCal_callback);
    sam_cmd_pub =  n.advertise<uxa_motion_control::SAMcmdMsg>("sam_cmd_sub",200);
    display_cmd_pub =  n.advertise<uxa_motion_control::displayCmdMsg>("display_cmd",200);
    sam_pid_pub = n.advertise<uxa_motion_control::SAMJointPIDMsg>("sam_PID",200);
    sam_torque_pub = n.advertise<uxa_motion_control::SAMJointTorqueMsg>("sam_torque",200);


    sam_pos12_pub=n.advertise<uxa_motion_control::SAMJointPos12Msg>("sam_pos12_sub",500);

    plot_pub=n.advertise<uxa_motion_control::uxaParameter_displayMsg>("plot_channel",500);

    PID_control_init();
    ROS_INFO("%s", "setup_motion_control");

    for(unsigned char i=0; i<12;i++)
    {
        samPos12.SAMPos12[i]=samPos12_hardware[i];
        samPos12.SAMMode[i]=1;
    }
    for(unsigned char i=12; i<NUM_OF_SAM_;i++)
    {
        samPos12.SAMPos12[i]=samPos12_hardware[i];
        samPos12.SAMMode[i]=0;
    }
    samPos12.SAMMode[22]=1;

    memset(currentSamPos12,'\0',25);

    task.id=TASK_IDLE_;
    task.preId=TASK_IDLE_;
    while(ros::ok())
    {

        if(FlagTimer.Hz_100)
        {
            FlagTimer.Hz_100=0;
            //====================
            task_handle();
            controller_handle();

        }
        if(FlagTimer.Hz_125)
        {
            FlagTimer.Hz_125=0;
            //===============

        }

        if(FlagTimer.Hz_50)
        {
            FlagTimer.Hz_50=0;
            //================
            //                unsigned char Trans_chr[1] ={ '0'};
            //                mySAM->Send_Serial_String(mySAM->Serial, Trans_chr, 1);
            //                  ROS_INFO("%s", "getFilteredData_50Hz");
            //            task_handle();
            //            controller_handle();

        }


        //==========================================


        ros::spinOnce();
        loop_rate.sleep();
        Timer_handler();
    }
    return 0;
}



void controller_handle(){

    if(sys_flag.feedback_sensor_available){
        sys_flag.feedback_sensor_available=0;

        // using sensor feedback (imu)
        if(sys_flag.controller_body_pitch){
            pid_body_pitch.PID_type_5(imu.roll,0.8);
        }
        // using sensor feedback (zmp)
        if(sys_flag.controller_ankle_left){
            //            pid_ankle_left.PID_type_5(leftZMP.posX,0.8);
//             pid_ankle_right.PID_type_5(leftZMP.posX,0.8);
//             pid_ankle_left.set_point=pid_ankle_right.output;
            pid_ankle_left.PID_type_5(imu.pitch,0.8);
            pid_body_roll_8_9.set_point=pid_ankle_left.output;

        }


        if(sys_flag.controller_ankle_right){
            pid_ankle_right.PID_type_5(rightZMP.posX,0.8);
        }
    }

    if(sys_flag.feedback_sam_available){
        sys_flag.feedback_sam_available=0;
        // using sam feedback
        if(sys_flag.controller_body_roll){

            if(currentSamAvail[1]&currentSamAvail[9]){
                double angle_1=-((double)currentSamPos12[1]-(double)samPos12_hardware[1])*pos12bitTodegree;
                double angle_9=((double)currentSamPos12[9]-(double)samPos12_hardware[9])*pos12bitTodegree;
                pid_body_roll_0_1.PID_type_5(angle_1,0.8);
                pid_body_roll_8_9.PID_type_5(angle_9,0.8);



                if((task.id==TASK_STEP_TEST)&&(task.scene==1)){
                    if(abs(pid_body_roll_0_1.er)<1){
                        currentScene.flag.finish=1;
                        cout<< "control stable"<< endl;
                    }

                }


                if((task.id==TASK_STEP_TEST)&&(task.scene==3)){
                    if(rightZMP.amp>50){
                        currentScene.flag.finish=1;
                        currentScene.flag.enable=0;
                        cout<< "foot contact"<< endl;
                    }

                }

            }

        }
    }

    if(sys_flag.feedback_tfCal_available){
        sys_flag.feedback_tfCal_available=0;

        //using tfCal feedback
        if(sys_flag.controller_foot_place){
            pid_foot_place.PID_type_5(footsDistance,0.8);



        }
    }


    unsigned char controller_on=0;
    if(sys_flag.controller_ankle_left|sys_flag.controller_ankle_right|sys_flag.controller_body_pitch|sys_flag.controller_body_roll|sys_flag.controller_foot_place){
        controller_on=1;
        //========



        controller_output[6]=-pid_body_pitch.output;
        controller_output[2]=-pid_body_pitch.output*0.6;
        controller_output[3]=pid_body_pitch.output*0.6;
        controller_output[7]=pid_body_pitch.output;
        //========
        controller_output[1]=-pid_body_roll_0_1.output;
//        if(leftZMP.amp>50)
//        {
//            controller_output[1]-=pid_ankle_left.output;
//        }

        controller_output[0]=-pid_body_roll_0_1.output;
        if(rightZMP.amp>50)
        {
            controller_output[0]-=pid_ankle_right.output;
        }

        controller_output[9]=pid_body_roll_8_9.output;//+pid_foot_place.output+pid_ankle_left.output;
        controller_output[8]=pid_body_roll_8_9.output-pid_foot_place.output;

        plotMsg.CN1= pid_body_roll_8_9.set_point;
        plotMsg.CN2=pid_body_roll_8_9.output;
        plotMsg.CN3=pid_ankle_left.set_point;
//        plotMsg.CN4=pid_body_roll_8_9.er;
        plot_pub.publish(plotMsg);

    }

    if(currentScene.flag.enable|controller_on){
        for (unsigned char i=0;i<NUM_OF_SAM_;i++){
            samPos12.SAMPos12[i]=(unsigned int)(pos12_double[i]+controller_output[i]);
            currentControlledSamPos12[i]=samPos12.SAMPos12[i];
        }
        if(sys_flag.enable_sam)
            sam_pos12_pub.publish(samPos12);
    }

}


void task_handle(){
    switch (task.id){
    case TASK_INIT_F_IDLE:
        if(task.startFlag==0){
            task.startFlag=1;
            ROS_INFO("task init: start");
            //===============

        }

        else if(task.finishFlag){
            ROS_INFO("task init: finish");
            task.finishFlag=0;
            task.startFlag=0;

            task.setup_para=0;
            task.setup_para_count=0;
            task.preId=TASK_INIT_F_IDLE;
            task.id=TASK_IDLE_;
            cout <<"finish init task "<<endl;
        }

        else if(sys_flag.feedback_sam_available)
        {
            sys_flag.feedback_sam_available=0;
            //====================================
            unsigned char count=0;
            for(unsigned char i=0;i<NUM_OF_SAM_;i++){
                if(currentSamAvail[i]==1)
                    count++;
            }

            if((count==13)&&(currentSamPos12[0]>0))
            {
                task.finishFlag=1;
                cout <<"init task done"<<endl;
                for (unsigned char i=0;i<NUM_OF_SAM_;i++)
                {
                    cout <<currentSamPos12[i]<<":";
                }
                cout<<endl;

                //                for(unsigned char i=0;i<12;i++)
                //                {
                //                    pos12[i]=currentSamPos12[i];
                //                    angle[i]=((double)pos12[i]-(double)samPos12_offset[i])*pos12bitTorad;
                //                }
            }
            else{
                //                sleep(1);
                cout <<"init error: "<<(int)count<<endl;
            }
        }

        break;
    case TASK_SITDOWN_F_INIT:
        if(task.startFlag==0){
            if(task.setup_para==0)
            {
                ROS_INFO("task sitdown: setup parameter ...");
                switch(task.setup_para_count){
                case 0:
                    for(unsigned char i=0;i<12;i++){
                        samPID.P[i]=sitdown_samP[i];
                        samPID.D[i]=sitdown_samD[i];
                        samPID.I[i]=0;
                    }
                    sam_pid_pub.publish(samPID);
                    task.setup_para_count++;
                    break;
                case 1:
                    for(unsigned char i=0;i<12;i++){
                        samPID.P[i]=sitdown_samP[i];
                        samPID.D[i]=sitdown_samD[i];
                        samPID.I[i]=0;
                    }
                    sam_pid_pub.publish(samPID);
                    task.setup_para_count++;
                    break;
                case 2:
                    for(unsigned char i=0;i<12;i++){
                        samTorque.Torque[i]=sitdown_averageTorq[i];
                    }
                    sam_torque_pub.publish(samTorque);
                    task.setup_para_count++;
                    break;
                case 3:
                    for(unsigned char i=0;i<12;i++){
                        samTorque.Torque[i]=sitdown_averageTorq[i];
                    }
                    sam_torque_pub.publish(samTorque);
                    task.setup_para_count++;
                    break;
                default:
                    task.setup_para=1;
                    task.setup_para_count=0;
                    break;
                }

            }
            else{
                task.startFlag=1;
                task.setup_para_count=0;
                task.setup_para=0;
                ROS_INFO("task sitdown: start");
                int beginpose[NUM_OF_SAM_];
                currentScene.mapHardToSoftPose(currentSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(400,beginpose,pose_sitdown);
            }
        }
        else if(currentScene.flag.finish)
        {
            currentScene.flag.finish=0;
            task.finishFlag=1;
        }
        else if(task.finishFlag){
            ROS_INFO("task sitdown: finish");
            task.finishFlag=0;
            task.startFlag=0;
            task.preId=TASK_SITDOWN_F_INIT;
            task.id=TASK_IDLE_;
            //==================
            samPos12.SAMMode[22]=0;
        }
        break;
    case TASK_STANDING_F_SITDOWN:
        if(task.startFlag==0){

            if(task.setup_para==0)
            {
                ROS_INFO("task sitdown: setup parameter ...");
                switch(task.setup_para_count){
                case 0:
                    for(unsigned char i=0;i<12;i++){
                        samPID.P[i]=standing_samP[i];
                        samPID.D[i]=standing_samD[i];
                        samPID.I[i]=0;
                    }
                    sam_pid_pub.publish(samPID);
                    task.setup_para_count++;
                    break;
                case 1:
                    for(unsigned char i=0;i<12;i++){
                        samPID.P[i]=standing_samP[i];
                        samPID.D[i]=standing_samD[i];
                        samPID.I[i]=0;
                    }
                    sam_pid_pub.publish(samPID);
                    task.setup_para_count++;
                    break;
                case 2:
                    for(unsigned char i=0;i<12;i++){
                        samTorque.Torque[i]=standing_averageTorq[i];
                    }
                    sam_torque_pub.publish(samTorque);
                    task.setup_para_count++;
                    break;
                case 3:
                    for(unsigned char i=0;i<12;i++){
                        samTorque.Torque[i]=standing_averageTorq[i];
                    }
                    sam_torque_pub.publish(samTorque);
                    task.setup_para_count++;
                    break;
                default:
                    task.setup_para=1;
                    task.setup_para_count=0;
                    break;
                }

            }else{
                ROS_INFO("task standing: start");
                task.setup_para_count=0;
                task.setup_para=0;
                task.startFlag=1;
                task.scene=0;
                currentScene.flag.finish=1;
                PID_reset();
                /*
             * Your code begin from here
             */
            }

        }
        else if(currentScene.flag.finish)
        {
            switch(task.scene){
            case 0:
                currentScene.setUpMyScene(numOfFrames_standing[task.scene],pose_standing_1,pose_standing_2);
                break;
            case 1:
                currentScene.setUpMyScene(numOfFrames_standing[task.scene],pose_standing_2,pose_standing_3);
                break;
            default:
                task.finishFlag=1;
                break;
            }
            currentScene.flag.finish=0;
            task.scene++;
        }
        else if(task.finishFlag){

            task.finishFlag=0;
            task.startFlag=0;
            task.id=TASK_IDLE_;
            task.preId=TASK_STANDING_F_SITDOWN;
            //            sleep(1);
            ROS_INFO("task standing: finish");
        }
        break;

    case TASK_STANDINGINIT_F_STANDING:
        if(task.startFlag==0){
            task.startFlag=1;
            currentScene.setUpMyScene(300,pose_standing_3,pose_init_walking);
            ROS_INFO("task standinginit: start");
        }
        else if(currentScene.flag.finish)
        {
            currentScene.flag.finish=0;
            task.finishFlag=1;
        }
        else if(task.finishFlag){
            task.finishFlag=0;
            task.startFlag=0;
            task.preId=TASK_STANDINGINIT_F_STANDING;
            task.id=TASK_IDLE_;
            ROS_INFO("task standinginit: finish");
            // PID_reset();
            // sys_flag.controller_body_pitch=1;
        }
        break;

    case TASK_STANDING_F_STANDINGINIT:
        if(task.startFlag==0){
            task.startFlag=1;
            //===============

            int beginpose[NUM_OF_SAM_];
            currentScene.mapHardToSoftPose(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
            currentScene.setUpMyScene(300,beginpose,pose_standing_3);
            //              currentScene.setUpMyScene(300,pose_init_walking,pose_standing_3);

            PID_reset();
            sys_flag.controller_body_pitch=0;
            sys_flag.controller_body_roll=0;
            sys_flag.controller_ankle_left=0;
            sys_flag.controller_ankle_right=0;
            sys_flag.controller_foot_place=0;
            ROS_INFO("task standing from standing init: start");
        }
        else if(currentScene.flag.finish)
        {
            currentScene.flag.finish=0;
            task.finishFlag=1;
        }
        else if(task.finishFlag){
            task.finishFlag=0;
            task.startFlag=0;
            task.preId=TASK_STANDING_F_STANDINGINIT;
            task.id=TASK_IDLE_;
            ROS_INFO("task standing from standing init: finish");
        }
        break;

    case TASK_SITDOWN_F_STANDING:
        if(task.startFlag==0){
            ROS_INFO("task sitdown from standing: start");
            task.startFlag=1;
            task.scene=0;
            currentScene.flag.finish=1;
            /*
             * Your code begin from here
             */
        }
        else if(currentScene.flag.finish)
        {
            switch(task.scene){
            case 0:
                currentScene.setUpMyScene(300,pose_standing_3,pose_standing_2);
                break;
            case 1:
                currentScene.setUpMyScene(300,pose_standing_2,pose_standing_1);
                break;
            default:
                task.finishFlag=1;
                break;
            }

            currentScene.flag.finish=0;
            task.scene++;
        }
        else if(task.finishFlag){
            task.finishFlag=0;
            task.startFlag=0;
            task.id=TASK_IDLE_;
            task.preId=TASK_SITDOWN_F_STANDING;
            sleep(1);
            ROS_INFO("task standing: finish");
        }
        break;
    case TASK_STEP_TEST:
        if(task.startFlag==0){
            ROS_INFO("task step from init: start");
            task.startFlag=1;
            task.scene=0;
            currentScene.flag.finish=1;
            /*
             * Your code begin from here
             */
        }
        else if(currentScene.flag.finish)
        {
            switch(task.scene){
            int beginpose[NUM_OF_SAM_];
            case 0:
                //                int beginpose[NUM_OF_SAM_];
                //                currentScene.mapHardToSoftPose(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                //                currentScene.setUpMyScene(numOfFrames_steping[0],beginpose,pose_step_2);
                //                PID_reset();
                pid_body_roll_0_1.set_point=-8;
                pid_body_roll_8_9.set_point=-6;
                pid_body_roll_0_1.KP=0;
                pid_body_roll_0_1.KI=10;

                pid_body_roll_8_9.KP=0;
                pid_body_roll_8_9.KI=10;

                sys_flag.controller_body_roll=1;
                cout<<"scene 0"<<endl;

                break;
            case 1:
                //                int beginpose[NUM_OF_SAM_];
                PID_reset();
                pid_body_roll_0_1.KP=10;
                pid_body_roll_0_1.KI=40;
                pid_body_roll_8_9.KP=10;
                pid_body_roll_8_9.KI=40;
                sys_flag.controller_foot_place=1;
                sys_flag.controller_ankle_left=1;
                pid_ankle_left.set_point=2;
                //                sys_flag.controller_ankle_left=1;
                //                pid_ankle_left.set_point=8;

                currentScene.mapHardToSoftPose(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(numOfFrames_steping[0],beginpose,pose_step_2);
                cout<<"scene 1"<<endl;
                //                PID_reset();
                //                sys_flag.controller_body_roll=0;

                //                currentScene.setUpMyScene(numOfFrames_steping[1],pose_step_2,pose_step_3);
                break;
//            case 2:
//                PID_reset();
//                currentScene.mapHardToSoftPose(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
//                currentScene.setUpMyScene(numOfFrames_steping[1],beginpose,pose_step_3);
//                cout<<"scene 2"<<endl;
//                break;
//            case 3:
//                PID_reset();
//                currentScene.mapHardToSoftPose(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
//                currentScene.setUpMyScene(100,beginpose,pose_step_3);
//                cout<<"scene 3"<<endl;
//                break;
            default:
                sys_flag.controller_foot_place=0;
                sys_flag.controller_ankle_left=0;
                task.finishFlag=1;
                break;
            }

            currentScene.flag.finish=0;
            task.scene++;
        }
        else if(task.finishFlag){
            ROS_INFO("task step: finish");
            task.finishFlag=0;
            task.startFlag=0;
            task.id=TASK_IDLE_;
            task.preId=TASK_STEP_TEST;
        }
        break;
    case TASK_STANDINGINIT_F_STEP:
        if(task.startFlag==0){
            task.startFlag=1;
            //===============

            int beginpose[NUM_OF_SAM_];
            currentScene.mapHardToSoftPose(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
            currentScene.setUpMyScene(300,beginpose,pose_init_walking);
            //              currentScene.setUpMyScene(300,pose_init_walking,pose_standing_3);
            PID_reset();
            sys_flag.controller_body_pitch=0;
            sys_flag.controller_body_roll=0;
            sys_flag.controller_ankle_left=0;
            sys_flag.controller_ankle_right=0;
            sys_flag.controller_foot_place=0;
            ROS_INFO("task standing init from step: start");
        }
        else if(currentScene.flag.finish)
        {
            currentScene.flag.finish=0;
            task.finishFlag=1;
        }
        else if(task.finishFlag){
            task.finishFlag=0;
            task.startFlag=0;
            task.preId=TASK_STANDINGINIT_F_STEP;
            task.id=TASK_IDLE_;
            ROS_INFO("task standing init from step: finish");
        }


        break;
    default:
        break;
    }

    if(currentScene.flag.enable){
        currentScene.frame++;

        if(currentScene.flag.delay){
            if(currentScene.flag.start==0){
                currentScene.flag.start=1;
                currentScene.frame=0;
            }
            else if(currentScene.frame>currentScene.numOfFrame)
            {
                currentScene.flag.enable=0;
                currentScene.flag.finish=1;
                currentScene.flag.delay=0;
            }
        }
        else{
            if(currentScene.flag.start==0){
                currentScene.flag.start=1;
                currentScene.frame=0;
                for(unsigned char i=0;i<NUM_OF_SAM_;i++)
                {
                    pos12_double[i]=*(currentScene.beginPose+i);
                    unsigned int b=*(currentScene.beginPose+i);
                    unsigned int a=*(currentScene.endPose+i);
                    delta_pos12[i]=((double)a-(double)b)/(double)currentScene.numOfFrame;
                }

                //                cout<<"pos12 double: ";
                //                for(unsigned char i=0;i<25;i++){
                //                    cout<<pos12_double[i]<<":";
                //                }
                //                cout<<endl;

                //                cout<<"delta_pose12: ";
                //                for(unsigned char i=0;i<25;i++){
                //                    cout<<delta_pos12[i]<<":";
                //                }
                //                cout<<endl;
            }
            else if(currentScene.frame<currentScene.numOfFrame)
            {
                for(unsigned char i=0;i<NUM_OF_SAM_;i++)
                {
                    pos12_double[i]+=delta_pos12[i];
                }
            }
            else{
                currentScene.flag.enable=0;
                currentScene.flag.finish=1;
            }

            //            if(currentScene.flag.enable){
            //                for (unsigned char i=0;i<NUM_OF_SAM_;i++){
            //                    samPos12.SAMPos12[i]=(unsigned int)pos12_double[i];
            //                    //                    currentControlledSamPos12[i]=samPos12.SAMPos12[i];
            //                }
            //                //                if(sys_flag.enable_sam)
            //                //                    sam_pos12_pub.publish(samPos12);
            //            }
        }
    }

    // update motion generator
    //    for (unsigned char i=0;i<NUM_OF_SAM_;i++){
    //        samPos12.SAMPos12[i]=(unsigned int)pos12_double[i];
    //    }
}

void Scene_class::setBeginPose(unsigned int *value)
{
    beginPose = value;
}

void Scene_class::setEndPose(unsigned int *value)
{
    endPose = value;
}

void Scene_class::setNumOfFrame(unsigned int value)
{
    numOfFrame = value;
}

void Scene_class::setDelayScene(unsigned int fr)
{
    this->setNumOfFrame(fr);
    this->frame=0;
    this->flag.start=0;
    this->flag.finish=0;
    this->flag.enable=1;
    this->flag.delay=1;
}

//unsigned int buf_beginPose[NUM_OF_SAM_];
//unsigned int buf_endPose[NUM_OF_SAM_];
void Scene_class::setUpMyScene(unsigned int fr, int *beginpose, int *endpose)
{
    mapSoftToHardPose(beginpose,this->buf_beginPose,NUM_OF_SAM_);
    mapSoftToHardPose(endpose,this->buf_endPose,NUM_OF_SAM_);
    this->setNumOfFrame(fr);
    this->setBeginPose(buf_beginPose);
    this->setEndPose(buf_endPose);
    this->frame=0;
    this->flag.start=0;
    this->flag.finish=0;
    this->flag.enable=1;
    this->flag.delay=0;

    //    cout<<"begin Pose:";
    //    for(unsigned char i=0;i<25;i++){
    //        cout<<(double)buf_beginPose[i]<<":";

    //    }
    //    cout<<endl;
    //    cout<<"endpose:";
    //    for(unsigned char i=0;i<25;i++){
    //        cout<<(double)buf_endPose[i]<<":";
    //    }
    //    cout<<endl;
}

void Scene_class::setFrame(unsigned int value)
{
    frame = value;
}

void Scene_class::mapSoftToHardPose( int *softwarePose, unsigned int *hardwarePose, unsigned char size)
{
    for(unsigned char i=0;i<size;i++)
    {
        *(hardwarePose+i)=(unsigned int)((int)*(softwarePose+i)+(int)samPos12_hardware[i]);
    }
}

void Scene_class::mapHardToSoftPose(unsigned int *hardwarePose, int *softwarePose, unsigned char size)
{
    for(unsigned char i=0;i<size;i++)
    {
        *(softwarePose+i)=(unsigned int)((int)*(hardwarePose+i)-(int)samPos12_hardware[i]);
    }
}
