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
#include "uxa_motion_control/uxaSetPIDJointMsg.h"

#include "uxa_motion_control/uxaMotionCommandMsg.h"






#define PID_SET_BODY_PITCH_         1
#define SETPOINT_SET_BODY_PITCH_    2

#define PID_SET_BODY_ROLL_          3
#define SETPOINT_SET_BODY_ROLL_     4

#define PID_IMPEDANCE_LEFT_         5
#define SETPOINT_IMPEDANCE_LEFT_    6

#define PID_IMPEDANCE_RIGHT_        7
#define SETPOINT_IMPEDANCE_RIGHT_   8

#define PID_ANKLE_LEFT_ROLL_        9
#define SETPOINT_ANKLE_LEFT_ROLL_   10

#define PID_ANKLE_RIGHT_ROLL_       11
#define SETPOINT_ANKLE_RIGHT_ROLL_  12

#define PID_ANKLE_LEFT_PITCH_       13
#define SETPOINT_ANKLE_LEFT_PITCH_  14

#define PID_ANKLE_RIGHT_PITCH_      15
#define SETPOINT_ANKLE_RIGHT_PITCH_ 16

#define PID_FOOT_PLACE_             17
#define SETPOINT_FOOT_PLACE_        18




ros::Publisher sensor_cmd_pub;
ros::Publisher sam_cmd_pub;
ros::Publisher display_cmd_pub;

ros::Publisher sam_pos12_pub;
ros::Publisher sam_pid_pub;
ros::Publisher sam_torque_pub;

uxa_motion_control::SAMJointPos12Msg samPos12;
uxa_motion_control::SAMJointPIDMsg samPID;
uxa_motion_control::SAMJointTorqueMsg samTorque;

ros::Publisher plot_pub;
uxa_motion_control::uxaParameter_displayMsg plotMsg;

// task follow oders
#define STEP_COM_TRANSFER_LEFT      0
#define STEP_RIGHT_LEG_UP           1
#define STEP_RIGHT_LEG_DOWN         2
#define STEP_IMPEDANCE_CONTROL_1    3
#define STEP_STABLE_WAIT_1          4
#define STEP_RETURN_INIT_1          5
#define STEP_COM_TRANSFER_RIGHT     6
#define STEP_LEFT_LEG_UP            7
#define STEP_LEFT_LEG_DOWN          8
#define STEP_IMPEDANCE_CONTROL_2    9
#define STEP_STABLE_WAIT_2          10
#define STEP_RETURN_INIT_2          11

void task_handle();
void controller_handle();
void sam_tfCal_callback(const uxa_motion_control::SAMtfCalMsg::ConstPtr& msg);
void sam_callback(const uxa_motion_control::SAMJointStateMsg::ConstPtr& msg);
void sensor_callback(const uxa_motion_control::dataSensorMsg::ConstPtr& msg);
void sub_function_cmd(const uxa_motion_control::motionCmdMsg::ConstPtr& msg);
void sub_function_walking(const uxa_motion_control::uxaMotionCommandMsg::ConstPtr& msg);
void sub_function_setPID(const uxa_motion_control::uxaSetPIDMsg::ConstPtr& msg);
void sub_function_setPIDJoints(const uxa_motion_control::uxaSetPIDJointMsg::ConstPtr& msg);
void PID_control_init();
void PID_reset();
void PID_joint_enable(unsigned char stt);


int main(int argc, char **argv){
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE_1000Hz_);

    sensor_cmd_pub =  n.advertise<uxa_motion_control::cmdSensorMsg>("sensor_sub",200);
    ros::Subscriber motion_cmd_sub=n.subscribe<uxa_motion_control::motionCmdMsg>("motion_cmd_sub",200,sub_function_cmd);
    ros::Subscriber setPID_sub =n.subscribe<uxa_motion_control::uxaSetPIDMsg>("setPID",200,sub_function_setPID);
    ros::Subscriber setPIDJoint_sub =n.subscribe<uxa_motion_control::uxaSetPIDJointMsg>("setPIDJoints",200,sub_function_setPIDJoints);

    ros::Subscriber sensor_data_sub =  n.subscribe<uxa_motion_control::dataSensorMsg>("sensor_pub",1000,sensor_callback);
    ros::Subscriber sam_joint_state_sub =  n.subscribe<uxa_motion_control::SAMJointStateMsg>("sam_pub",1000,sam_callback);
    ros::Subscriber sam_tfCal_sub = n.subscribe<uxa_motion_control::SAMtfCalMsg>("sam_tfCal_pub",500,sam_tfCal_callback);
    sam_cmd_pub =  n.advertise<uxa_motion_control::SAMcmdMsg>("sam_cmd_sub",200);
    display_cmd_pub =  n.advertise<uxa_motion_control::displayCmdMsg>("display_cmd",200);
    sam_pid_pub = n.advertise<uxa_motion_control::SAMJointPIDMsg>("sam_PID",200);
    sam_torque_pub = n.advertise<uxa_motion_control::SAMJointTorqueMsg>("sam_torque",200);

    ros::Subscriber uxa_motion_sub=n.subscribe<uxa_motion_control::uxaMotionCommandMsg>("uxa_walking_sub",200,sub_function_walking);

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
    memset(sys_flag.controller_joint,'\0',12);

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

        }


        //==========================================


        ros::spinOnce();
        loop_rate.sleep();
        Timer_handler();
    }
    return 0;
}

bool controller_on=0;

void controller_handle(){
    //    if(sys_flag.feedback_sam_available&&sys_flag.feedback_tfCal_available&&sys_flag.feedback_sensor_available){
    //        sys_flag.feedback_sam_available=0;
    //        sys_flag.feedback_tfCal_available=0;
    //        sys_flag.feedback_sensor_available=0;

    if(sys_flag.feedback_sensor_available){
        sys_flag.feedback_sensor_available=0;
        controller_on=0;
        // using sam feedback
        //        for(unsigned char i=0;i<12;i++){
        //            if(sys_flag.controller_joint[i]){
        //                pid_joint[i].set_point=posSetPointDegree[i];
        //            }
        //        }

        //        pid_joint[6].set_point=posSetPointDegree[6];
        //        pid_joint[7].set_point=posSetPointDegree[7];

        if(sys_flag.controller_body_pitch){
            controller_on=1;
//            pid_body_pitch.PID_type_5(imu.roll,0.9);
            pid_body_pitch.PID_type_5(imu.roll_rate,0.1);
            controller_output[7]=pid_body_pitch.output;
            controller_output[6]=-pid_body_pitch.output;
//                        controller_output[3]=-pid_body_pitch.output;
//                        controller_output[2]=pid_body_pitch.output;
        }

        if(sys_flag.controller_body_roll){

        }

        //        if(sys_flag.controller_ankle_left_roll){
        //            if(leftZMP.amp>40){

        //                if(pid_ankle_left_roll.flag_enable==0){
        //                    pid_ankle_left_roll.I_term=pid_ankle_left_roll.pre_output;
        //                }
        //                pid_ankle_left_roll.flag_enable=1;
        //                pid_ankle_left_roll.PID_type_5(leftZMP.posX,0.8);
        //                pid_joint[1].set_point=-pid_ankle_left_roll.output;
        //            }else{
        //                if(pid_ankle_left_roll.flag_enable)
        //                {
        //                    pid_ankle_left_roll.pre_output=pid_ankle_left_roll.output;
        //                    pid_ankle_left_roll.reset_parameters();
        //                }
        //                pid_ankle_left_roll.flag_enable=0;
        //                pid_joint[1].set_point=-pid_ankle_left_roll.pre_output;
        //            }
        //        }

        //        if(sys_flag.controller_ankle_right_roll){
        //            if(rightZMP.amp>40){
        //                if(pid_ankle_right_roll.flag_enable==0){
        //                    pid_ankle_right_roll.I_term=pid_ankle_right_roll.pre_output;
        //                }
        //                pid_ankle_right_roll.flag_enable=1;
        //                pid_ankle_right_roll.PID_type_5(rightZMP.posX,0.8);
        //                pid_joint[0].set_point=-pid_ankle_right_roll.output;
        //            }else{
        //                if(pid_ankle_right_roll.flag_enable)
        //                {
        //                    pid_ankle_right_roll.pre_output=pid_ankle_right_roll.output;
        //                    pid_ankle_right_roll.reset_parameters();
        //                }
        //                pid_ankle_right_roll.flag_enable=0;
        //                pid_joint[0].set_point=-pid_ankle_right_roll.pre_output;
        //            }
        //        }

        //        if(sys_flag.controller_ankle_left_pitch){
        //            if(leftZMP.amp>40){
        //                if(pid_ankle_left_pitch.flag_enable==0){
        //                    pid_ankle_left_pitch.I_term=pid_ankle_left_pitch.pre_output;
        //                }
        //                pid_ankle_left_pitch.flag_enable=1;
        //                pid_ankle_left_pitch.PID_type_5(leftZMP.posY,0.8);
        //                pid_joint[3].set_point=-pid_ankle_left_pitch.output;
        //            }else{
        //                if(pid_ankle_left_pitch.flag_enable)
        //                {
        //                    pid_ankle_left_pitch.pre_output=pid_ankle_left_pitch.output;
        //                    pid_ankle_left_pitch.reset_parameters();
        //                }
        //                pid_ankle_left_pitch.flag_enable=0;
        //                pid_joint[3].set_point=-pid_ankle_left_pitch.pre_output;
        //            }
        //        }

        //        if(sys_flag.controller_ankle_right_pitch){
        //            if(rightZMP.amp>40){

        //                if(pid_ankle_right_pitch.flag_enable==0){
        //                    pid_ankle_right_pitch.I_term=pid_ankle_right_pitch.pre_output;
        //                }
        //                pid_ankle_right_pitch.flag_enable=1;
        //                pid_ankle_right_pitch.PID_type_5(rightZMP.posY,0.8);
        //                pid_joint[2].set_point=pid_ankle_right_pitch.output;
        //            }else{
        //                if(pid_ankle_right_pitch.flag_enable)
        //                {
        //                    pid_ankle_right_pitch.pre_output=pid_ankle_right_pitch.output;
        //                    pid_ankle_right_pitch.reset_parameters();
        //                }
        //                pid_ankle_right_pitch.flag_enable=0;
        //                pid_joint[2].set_point=pid_ankle_right_pitch.pre_output;
        //            }
        //        }


        //        if(sys_flag.controller_impedance){
        //            pid_impedance_right.PID_type_5(rightZMP.amp,0.8);
        //            pid_impedance_left.PID_type_5(leftZMP.amp,0.8);
        //            pid_impedace_ouputIntegrate_right-=pid_impedance_right.output;
        //            pid_impedace_ouputIntegrate_left-=pid_impedance_left.output;


        //            if(task.id==TASK_STEP_TEST){
        //                //                if(taskStepCurrentScene==STEP_IMPEDANCE_CONTROL_1)
        //                //                {

        //                //                    pid_impedace_ouputIntegrate_right=constrain(pid_impedace_ouputIntegrate_right,-5,5);
        //                //                    pid_impedace_ouputIntegrate_left=constrain(pid_impedace_ouputIntegrate_left,-5,15);
        //                //                    pid_joint[4].set_point=pid_impedace_ouputIntegrate_right+posSetPointDegree[4];
        //                //                    pid_joint[2].set_point=-pid_impedace_ouputIntegrate_right*0.6+posSetPointDegree[2];
        //                //                    pid_joint[6].set_point=pid_impedace_ouputIntegrate_right*0.6+posSetPointDegree[6];

        //                //                    pid_joint[5].set_point=-pid_impedace_ouputIntegrate_left+posSetPointDegree[5];
        //                //                    pid_joint[3].set_point=pid_impedace_ouputIntegrate_left*0.6+posSetPointDegree[3];
        //                //                    pid_joint[7].set_point=-pid_impedace_ouputIntegrate_left*0.6+posSetPointDegree[7];
        //                //                }
        //                //                else if(taskStepCurrentScene==STEP_IMPEDANCE_CONTROL_2){

        //                //                    pid_impedace_ouputIntegrate_right=constrain(pid_impedace_ouputIntegrate_right,-5,15);
        //                //                    pid_impedace_ouputIntegrate_left=constrain(pid_impedace_ouputIntegrate_left,-5,5);

        //                //                    pid_joint[5].set_point=-pid_impedace_ouputIntegrate_left+posSetPointDegree[5];
        //                //                    pid_joint[3].set_point=pid_impedace_ouputIntegrate_left*0.6+posSetPointDegree[3];
        //                //                    pid_joint[7].set_point=-pid_impedace_ouputIntegrate_left*0.6+posSetPointDegree[7];

        //                //                    pid_joint[4].set_point=pid_impedace_ouputIntegrate_right+posSetPointDegree[4];
        //                //                    pid_joint[2].set_point=-pid_impedace_ouputIntegrate_right*0.6+posSetPointDegree[2];
        //                //                    pid_joint[6].set_point=pid_impedace_ouputIntegrate_right*0.6+posSetPointDegree[6];
        //                //                }
        //            }

        //        }

        //        //using tfCal feedback
        //        if(sys_flag.controller_foot_place){
        //            pid_foot_place.PID_type_5(footsDistance,0.8);
        //            if(pid_foot_place.output<0)
        //            {
        //                pid_foot_place.output=0;
        //            }
        //            if(task.id==TASK_STEP_TEST){
        //                if(taskStepCurrentScene==STEP_RIGHT_LEG_UP||taskStepCurrentScene==STEP_RIGHT_LEG_DOWN){
        //                    pid_joint[8].set_point=posSetPointDegree[8]-pid_foot_place.output;
        //                }
        //                else if(taskStepCurrentScene==STEP_LEFT_LEG_UP||taskStepCurrentScene==STEP_LEFT_LEG_DOWN){
        //                    pid_joint[9].set_point=posSetPointDegree[9]+pid_foot_place.output;
        //                }

        //            }



        //        }

        for(unsigned char i=0;i<12;i++){
            if(sys_flag.controller_joint[i]){
                pid_joint[i].PID_type_5(currentSAMPosDegree[i],0.8);
            }
        }


    }




    //        bool controller_on=0;
    //    for(unsigned char i=0;i<12;i++){
    //        if(sys_flag.controller_joint[i])
    //        {
    //            controller_on=1;
    //            controller_output[i]=pid_joint[i].output;
    //        }
    //    }


    if(sys_flag.wait_for_stable)
    {
        if(stable_pose_flag)
            stable_cycle++;
        else
            stable_cycle=0;

        if(stable_cycle>20)
        {
            currentScene.flag.finish=1;
            currentScene.flag.enable=0;
            cout<< "finish stable wait 1 "<< endl;

            sys_flag.wait_for_stable=0;
            stable_cycle=0;
        }

        //        plotMsg.CN12=stable_cycle;

    }

    if(sys_flag.sinunoid_motion){
        if(step_right_status==STEP_UP){
            plotMsg.CN7=0.1;
        }
        else if(step_right_status==STEP_DOWN){
            plotMsg.CN7=0.2;
        }
        else if(step_left_status==STEP_UP){
            plotMsg.CN7=0.3;
        }
        else if(step_left_status==STEP_DOWN){
            plotMsg.CN7=0.4;
        }else{
            plotMsg.CN7=0;
        }
    }

    plotMsg.CN1= foot_left_position[0]+foot_pos_x_offset;
    plotMsg.CN2=foot_left_position[1];
    plotMsg.CN3=foot_left_position[2]+BODY_HEIGHT_LEFT;

    plotMsg.CN4= foot_right_position[0]+foot_pos_x_offset;
    plotMsg.CN5= foot_right_position[1];
    plotMsg.CN6=foot_right_position[2]+BODY_HEIGHT_RIGHT;

//    plotMsg.CN1=imu.roll-BODY_PITCH_SETPOINT_STANDING;

    //    plotMsg.CN4= COMpos;
    //    plotMsg.CN5= pid_body_pitch.output;
    //    plotMsg.CN6=pid_body_pitch.fb;
    //    plotMsg.CN11=pid_body_pitch.set_point;

    //    plotMsg.CN7=posSetPointDegree[6];
    plotMsg.CN8=pid_body_pitch.output;
    plotMsg.CN9=pid_body_pitch.fb_filter;
    plotMsg.CN10=pid_body_pitch.fb;
    plotMsg.CN11=pid_body_pitch.D_term;
    plotMsg.CN12=pid_body_pitch.I_term;

    for(unsigned char i=0; i<12;i++){
        plotMsg.controller_output[i]=controller_output[i];
        plotMsg.posSetPointDegree[i]=posSetPointDegree[i];
    }
    plot_pub.publish(plotMsg);


    if(currentScene.flag.enable||controller_on){
        //        if(task.id==TASK_STEP_TEST){
        //            if(taskStepCurrentScene==STEP_RIGHT_LEG_DOWN){
        //                if(rightZMP.amp>60){
        //                    currentScene.flag.finish=1;
        //                    currentScene.flag.enable=0;
        //                    cout<< "foot contact scene 2"<< endl;
        //                    //                    plotMsg.CN9=1;
        //                }
        //                //                else
        //                //                    plotMsg.CN9=2;
        //            }
        //            else if(taskStepCurrentScene==STEP_LEFT_LEG_DOWN){
        //                if(leftZMP.amp>60){
        //                    currentScene.flag.finish=1;
        //                    currentScene.flag.enable=0;
        //                    cout<< "foot contact scene 6"<< endl;
        //                    //                    plotMsg.CN9=3;
        //                }
        //                //                else
        //                //                    plotMsg.CN9=4;
        //            }

        //            if(taskStepCurrentScene==STEP_STABLE_WAIT_1)
        //            {


        //                if(stable_pose_flag)
        //                    stable_cycle++;
        //                else
        //                    stable_cycle=0;

        //                if(stable_cycle>50)
        //                {
        //                    currentScene.flag.finish=1;
        //                    currentScene.flag.enable=0;
        //                    cout<< "finish stable wait 1 "<< endl;
        //                    //                    plotMsg.CN9=5;
        //                    stable_cycle=0;
        //                }
        //                //                else
        //                //                    plotMsg.CN9=6;

        //            }

        //            else if(taskStepCurrentScene==STEP_STABLE_WAIT_2)
        //            {


        //                if(stable_pose_flag)
        //                    stable_cycle++;
        //                else
        //                    stable_cycle=0;

        //                if(stable_cycle>50)
        //                {
        //                    currentScene.flag.finish=1;
        //                    currentScene.flag.enable=0;
        //                    cout<< "finish stable wait 2 "<< endl;
        //                    //                    plotMsg.CN9=7;
        //                    stable_cycle=0;
        //                }
        //                //                else
        //                //                    plotMsg.CN9=8;
        //            }
        //        }




        for (unsigned char i=0;i<NUM_OF_SAM_;i++){
            //              samPos12.SAMPos12[i]=(unsigned int)((posSetPointDegree[i]+controller_output[i])*degreeToPose12+(double)samPos12_hardware[i]);
            samPos12.SAMPos12[i]=(unsigned int)(posSetPointDegree[i]*degreeToPose12+(double)samPos12_hardware[i]+controller_output[i]);

            if(samPos12.SAMMode[i]==1)
                currentControlledSamPos12[i]=samPos12.SAMPos12[i];
            else if(  samPos12.SAMMode[i]==2)
                currentControlledSamPos12[i]=(unsigned int)(currentSAMPosDegree[i]*degreeToPose12+(double)samPos12_hardware[i]);
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
            ROS_INFO("task init: start...");
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

            task.finishFlag=1;
            cout <<"init task done"<<endl;
            for (unsigned char i=0;i<NUM_OF_SAM_;i++)
            {
                if(currentSamAvail[i])
                    cout <<currentSAMPosDegree[i]<<":";
            }
            cout<<endl;

            //                for(unsigned char i=0;i<12;i++)
            //                {
            //                    pos12[i]=currentSamPos12[i];
            //                    angle[i]=((double)pos12[i]-(double)samPos12_offset[i])*pos12bitTorad;
            //                }
            //            }
            //            else{
            //                cout <<"init error: "<<(int)count<<endl;
            //            }
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
                //                int beginpose[NUM_OF_SAM_];
                //                currentScene.mapHardToSoftPose(currentSamPos12,beginpose,NUM_OF_SAM_);
                //                currentScene.setUpMyScene(400,beginpose,pose_sitdown);
                currentScene.setUpMyScene(400,currentSAMPosDegree,pose_sitdown);
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
        }
        break;

    case TASK_STANDING_F_STANDINGINIT:
        if(task.startFlag==0){
            task.startFlag=1;
            //===============

            double beginpose[NUM_OF_SAM_];
            currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
            currentScene.setUpMyScene(300,beginpose,pose_standing_3);


            PID_reset();
            //            sys_flag.controller_body_pitch=0;
            //            sys_flag.controller_body_roll=0;
            //            sys_flag.controller_ankle_left=0;
            //            sys_flag.controller_ankle_right=0;
            //            sys_flag.controller_foot_place=0;

            //            sys_flag.controller_joint_0=0;
            //            sys_flag.controller_joint_1=0;
            //            sys_flag.controller_joint_8=0;
            //            sys_flag.controller_joint_9=0;
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
            taskStepCurrentScene=task.scene;
            double beginpose[NUM_OF_SAM_];
            switch(task.scene){
            case STEP_COM_TRANSFER_LEFT:
                PID_joint_enable(1);

                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(100,beginpose,pose_step_comLeft);

                //                pid_joint[0].set_point=6;
                //                pid_joint[1].set_point=6;
                //                pid_joint[9].set_point=-6;
                //                pid_joint[8].set_point=-6;

                //                pid_joint[0].KP=0;
                //                pid_joint[0].KI=40;
                //                pid_joint[1].KP=0;
                //                pid_joint[1].KI=40;
                //                pid_joint[8].KP=0;
                //                pid_joint[8].KI=40;
                //                pid_joint[9].KP=0;
                //                pid_joint[9].KI=40;
                //                sys_flag.controller_joint[0]=1;
                //                sys_flag.controller_joint[1]=1;
                //                sys_flag.controller_joint[8]=1;
                //                sys_flag.controller_joint[9]=1;

                //                sys_flag.controller_foot_place=1;
                //                pid_foot_place.set_point=0.1;

                cout<<"scene com transfer left"<<endl;

                break;
            case STEP_RIGHT_LEG_UP:

                PID_reset();

                //                pid_joint_0.KP=0;
                //                pid_joint_0.KI=10;
                //                pid_joint_1.KP=0;
                //                pid_joint_1.KI=60;

                pid_joint[8].KI=200;
                pid_joint[9].KI=200;

                sys_flag.controller_foot_place=1;
                pid_foot_place.set_point=0.12;
                pid_foot_place.I_term=-pid_joint[8].set_point;


                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(numOfFrames_steping[0],beginpose,pose_step_rightUp);
                cout<<"scene right leg up"<<endl;
                break;
            case STEP_RIGHT_LEG_DOWN:

                PID_reset();
                //                sys_flag.controller_ankle_right_pitch=1;
                //                sys_flag.controller_ankle_right_roll=1;
                //                pid_ankle_right_pitch.pre_output=0;
                //                pid_ankle_right_roll.pre_output=0;

                //                sys_flag.controller_ankle_left_pitch=1;
                //                                sys_flag.controller_ankle_left_roll=1;
                //                pid_ankle_left_pitch.pre_output=0;
                //                                pid_ankle_left_roll.pre_output=0;

                pid_foot_place.I_term=-pid_joint[8].set_point;
                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(numOfFrames_steping[1],beginpose,pose_step_rightDown);
                cout<<"scene right leg down"<<endl;
                break;

            case STEP_IMPEDANCE_CONTROL_1:
                PID_reset();
                pid_foot_place.I_term=-pid_joint[8].set_point;
                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(20,beginpose,pose_step_impedance_1);

                //                PID_reset();
                //                pid_impedance_right.set_point=100;//120;
                //                pid_impedance_left.set_point=100;//270;
                //                sys_flag.controller_impedance=1;
                //                                pid_foot_place.I_term=-pid_joint[8].set_point;
                //                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                //                currentScene.setUpMyScene(30,beginpose,pose_step_rightDown);
                cout<<"scene impedance control 1"<<endl;
                break;
            case STEP_STABLE_WAIT_1:
                cout<<"scene stable 1"<<endl;
                break;

            case STEP_RETURN_INIT_1:
                PID_reset();
                //                PID_joint_enable(0);
                //                sys_flag.controller_foot_place=0;
                pid_foot_place.I_term=-pid_joint[8].set_point;
                sys_flag.controller_impedance=0;

                //                sys_flag.controller_ankle_right_pitch=0;
                //                sys_flag.controller_ankle_right_roll=0;
                //                pid_ankle_right_pitch.pre_output=0;
                //                pid_ankle_right_roll.pre_output=0;

                //                sys_flag.controller_ankle_left_pitch=0;
                //                sys_flag.controller_ankle_left_roll=0;
                //                pid_ankle_left_pitch.pre_output=0;
                //                pid_ankle_left_roll.pre_output=0;
                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(100,beginpose,pose_step_init);
                cout<<"scene return to init  pose"<<endl;
                break;

            case STEP_COM_TRANSFER_RIGHT:
                PID_joint_enable(1);
                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(100,beginpose,pose_step_comRight);


                cout<<"scene COM transfer to right"<<endl;

                break;

            case STEP_LEFT_LEG_UP:

                PID_reset();
                //                pid_joint[8].KI=200;
                //                pid_joint[9].KI=80;
                sys_flag.controller_foot_place=1;
                pid_foot_place.set_point=0.13;
                pid_foot_place.I_term=pid_joint[9].set_point;

                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(numOfFrames_steping[0],beginpose,pose_step_leftUp);
                cout<<"scene left leg up"<<endl;
                break;

            case  STEP_LEFT_LEG_DOWN:
                PID_reset();
                //                sys_flag.controller_ankle_right_pitch=1;
                //                sys_flag.controller_ankle_right_roll=1;
                //                pid_ankle_right_pitch.pre_output=0;
                //                pid_ankle_right_roll.pre_output=0;

                //                sys_flag.controller_ankle_left_pitch=1;
                //                sys_flag.controller_ankle_left_roll=1;
                //                pid_ankle_left_pitch.pre_output=0;
                //                pid_ankle_left_roll.pre_output=0;
                pid_foot_place.I_term=pid_joint[9].set_point;
                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(numOfFrames_steping[1],beginpose,pose_step_leftDown);
                cout<<"scene left leg down"<<endl;
                break;
            case STEP_IMPEDANCE_CONTROL_2:
                PID_reset();
                pid_foot_place.I_term=pid_joint[9].set_point;
                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(20,beginpose,pose_step_impedance_2);
                //                PID_reset();
                //                pid_impedance_right.set_point=100;//300;
                //                pid_impedance_left.set_point=100;//150;
                //                sys_flag.controller_impedance=1;

                //                pid_joint[0].set_point=0;
                //                  pid_joint[1].set_point=0;
                //                    pid_joint[8].set_point=0;
                //                      pid_joint[9].set_point=0;

                //                pid_foot_place.I_term=pid_joint[9].set_point;
                //                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                //                currentScene.setUpMyScene(100,beginpose,pose_step_leftDown);
                cout<<"scene impedance control 2"<<endl;
                break;
            case STEP_STABLE_WAIT_2:
                cout<<"scene stable 2"<<endl;
                break;
            case STEP_RETURN_INIT_2:
                PID_reset();
                PID_joint_enable(0);
                sys_flag.controller_foot_place=0;
                //                sys_flag.controller_impedance=0;
                //                sys_flag.controller_ankle_right_pitch=0;
                //                sys_flag.controller_ankle_right_roll=0;
                //                pid_ankle_right_pitch.pre_output=0;
                //                pid_ankle_right_roll.pre_output=0;

                //                sys_flag.controller_ankle_left_pitch=0;
                //                sys_flag.controller_ankle_left_roll=0;
                //                pid_ankle_left_pitch.pre_output=0;
                //                pid_ankle_left_roll.pre_output=0;
                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(100,beginpose,pose_step_init);
                cout<<"scene return to init pose"<<endl;
                break;

            default:
                //                sys_flag.controller_foot_place=0;
                //                sys_flag.controller_joint_0=0;
                //                sys_flag.controller_joint_1=0;
                //                sys_flag.controller_joint_8=0;
                //                sys_flag.controller_joint_9=0;

                //                                sys_flag.controller_ankle_left=0;
                //                sys_flag.controller_joint_0=0;
                //                sys_flag.controller_joint_1=0;
                //                sys_flag.controller_joint_8=0;
                //                sys_flag.controller_joint_9=0;
                task.finishFlag=1;
                break;
            }

            currentScene.flag.finish=0;// a scene must be finish outside this region
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
        //    case TASK_STANDINGINIT_F_STEP:
        //        if(task.startFlag==0){
        //            task.startFlag=1;
        //            //===============

        //            int beginpose[NUM_OF_SAM_];
        //            currentScene.mapHardToSoftPose(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
        //            currentScene.setUpMyScene(300,beginpose,pose_init_walking);
        //            //              currentScene.setUpMyScene(300,pose_init_walking,pose_standing_3);
        //            PID_reset();
        //            sys_flag.controller_body_pitch=0;
        //            sys_flag.controller_body_roll=0;
        //            sys_flag.controller_ankle_left=0;
        //            sys_flag.controller_ankle_right=0;
        //            sys_flag.controller_foot_place=0;
        //            ROS_INFO("task standing init from step: start");
        //        }
        //        else if(currentScene.flag.finish)
        //        {
        //            currentScene.flag.finish=0;
        //            task.finishFlag=1;
        //        }
        //        else if(task.finishFlag){
        //            task.finishFlag=0;
        //            task.startFlag=0;
        //            task.preId=TASK_STANDINGINIT_F_STEP;
        //            task.id=TASK_IDLE_;
        //            ROS_INFO("task standing init from step: finish");
        //        }


        //        break;
    case TASK_STEP_SINUNOID:
        if(task.startFlag==0){

            if(task.setup_para==0)
            {
                ROS_INFO("task step sinunoid: start");
                switch(task.setup_para_count){
                case 0:
                    for(unsigned char i=0;i<12;i++){
                        samPID.P[i]=steping_samP[i];
                        samPID.D[i]=steping_samD[i];
                        samPID.I[i]=0;
                    }
                    sam_pid_pub.publish(samPID);
                    task.setup_para_count++;
                    break;
                case 1:
                    for(unsigned char i=0;i<12;i++){
                        samPID.P[i]=steping_samP[i];
                        samPID.D[i]=steping_samD[i];
                        samPID.I[i]=0;
                    }
                    sam_pid_pub.publish(samPID);
                    task.setup_para_count++;
                    break;
                case 2:
                    for(unsigned char i=0;i<12;i++){
                        samTorque.Torque[i]=steping_averageTorq[i];
                    }
                    sam_torque_pub.publish(samTorque);
                    task.setup_para_count++;
                    break;
                case 3:
                    for(unsigned char i=0;i<12;i++){
                        samTorque.Torque[i]=steping_averageTorq[i];
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

                task.startFlag=1;
                task.scene=0;
                currentScene.flag.finish=1;

                /*
                 * Your code begin from here
                 */
                sys_flag.sinunoid_motion=0;
                step_timer=0;
            }

        }
        else if(currentScene.flag.finish)
        {

            switch(task.scene){
            case 0:
                double beginpose[NUM_OF_SAM_];

                foot_left_position[0]=-foot_pos_x_offset;
                foot_left_position[1]=AcomY*sin(phiY_left)+foot_left_posY_offset;
                double b;
                b=foot_pos_z_amp*sin(phiZ_left);
                if(b<0)
                    b=0;
                foot_left_position[2]=-BODY_HEIGHT_LEFT+b;

                foot_right_position[0]=-foot_pos_x_offset;
                foot_right_position[1]=AcomY*sin(phiY_right)-foot_right_posY_offset;
                b=foot_pos_z_amp*sin(phiZ_right);
                if(b<0)
                    b=0;
                foot_right_position[2]=-BODY_HEIGHT_RIGHT+b;

                inverseKinematic(foot_left_mat,foot_left_position[0],foot_left_position[1],foot_left_position[2],calAngle);
                pose_step_sinunoid_init[1]=calAngle[5]*180/M_PI;
                pose_step_sinunoid_init[3]=-calAngle[4]*180/M_PI;
                pose_step_sinunoid_init[5]=-calAngle[3]*180/M_PI;
                pose_step_sinunoid_init[7]=calAngle[2]*180/M_PI-body_tilt_offset;
                pose_step_sinunoid_init[9]=calAngle[1]*180/M_PI;
                pose_step_sinunoid_init[11]=calAngle[0]*180/M_PI;
                inverseKinematic(foot_right_mat,foot_right_position[0],foot_right_position[1],foot_right_position[2],calAngle);
                pose_step_sinunoid_init[0]=calAngle[5]*180/M_PI;
                pose_step_sinunoid_init[2]=calAngle[4]*180/M_PI;
                pose_step_sinunoid_init[4]=calAngle[3]*180/M_PI;
                pose_step_sinunoid_init[6]=-calAngle[2]*180/M_PI+body_tilt_offset;
                pose_step_sinunoid_init[8]=calAngle[1]*180/M_PI;
                pose_step_sinunoid_init[10]=calAngle[0]*180/M_PI;

                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(200,beginpose,pose_step_sinunoid_init);

                break;
            case 1:// step in left

                //                sys_flag.controller_body_pitch=1;
                setUp_stepMotion(STEP_MODE_INPLACE,STEP_LEGID_LEFT,0);
                break;
            case 2:
                sys_flag.wait_for_stable=1;
                cout<<"scene stable 2"<<endl;
                break;
            case 3://step right
                setUp_stepMotion(STEP_MODE_FORWARD_IN,STEP_LEGID_RIGHT,0);
                break;
            case 4:
                sys_flag.wait_for_stable=1;
                cout<<"scene stable 4"<<endl;
                break;
            case 5://step left
                setUp_stepMotion(STEP_MODE_FORWARD_NORMAL,STEP_LEGID_LEFT,0);
                break;
            case 6:
                sys_flag.wait_for_stable=1;
                cout<<"scene stable 6"<<endl;
                break;
            case 7://step out right
                setUp_stepMotion(STEP_MODE_FORWARD_NORMAL,STEP_LEGID_RIGHT,0);
                break;
            case 8:
                sys_flag.wait_for_stable=1;
                cout<<"scene stable 8"<<endl;
                break;
            case 9://step out right
                setUp_stepMotion(STEP_MODE_FORWARD_OUT,STEP_LEGID_LEFT,0);
                break;
                //            case 10:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 10"<<endl;
                //                break;
                //            case 11://step out right
                //                setUp_stepMotion(STEP_MODE_NORMAL,STEP_LEGID_LEFT,0);
                //                break;
                //            case 12:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 12"<<endl;
                //                break;

                //            case 13://step out right
                //                setUp_stepMotion(STEP_MODE_NORMAL,STEP_LEGID_RIGHT,0);
                //                break;
                //            case 14:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 10"<<endl;
                //                break;

                //            case 15://step out right
                //                setUp_stepMotion(STEP_MODE_NORMAL,STEP_LEGID_LEFT,0);
                //                break;
                //            case 16:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 12"<<endl;
                //                break;

                //            case 17://step out right
                //                setUp_stepMotion(STEP_MODE_NORMAL,STEP_LEGID_RIGHT,0);
                //                break;
                //            case 18:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 10"<<endl;
                //                break;

                //            case 19://step out right
                //                setUp_stepMotion(STEP_MODE_NORMAL,STEP_LEGID_LEFT,0);
                //                break;
                //            case 20:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 12"<<endl;
                //                break;

                //            case 21://step out right
                //                setUp_stepMotion(STEP_MODE_OUT,STEP_LEGID_RIGHT,0);
                //                break;
                //            case 22:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 14"<<endl;
                //                break;
                //            case 23://step out right
                //                setUp_stepMotion(STEP_MODE_ROTATE,STEP_LEGID_LEFT,9);
                //                break;
                //            case 24:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 16"<<endl;
                //                break;
                //            case 25://step out right
                //                setUp_stepMotion(STEP_MODE_ROTATE,STEP_LEGID_LEFT,9);
                //                break;
                //            case 26:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 18"<<endl;
                //                break;
                //            case 27://step out right
                //                setUp_stepMotion(STEP_MODE_ROTATE,STEP_LEGID_LEFT,9);
                //                break;

                //            case 28:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 18"<<endl;
                //                break;
                //            case 29://step out right
                //                setUp_stepMotion(STEP_MODE_ROTATE,STEP_LEGID_LEFT,9);
                //                break;

                //            case 30:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 18"<<endl;
                //                break;
                //            case 31://step out right
                //                setUp_stepMotion(STEP_MODE_ROTATE,STEP_LEGID_LEFT,9);
                //                break;
                //            case 32:
                //                sys_flag.wait_for_stable=1;
                //                cout<<"scene stable 18"<<endl;
                //                break;
            default:
                sys_flag.sinunoid_motion=0;
                flag_enable_step_in=0;
                task.finishFlag=1;
                sys_flag.controller_body_pitch=0;
                break;
            }

            currentScene.flag.finish=0;
            task.scene++;

        }
        else if(task.finishFlag){

            task.finishFlag=0;
            task.startFlag=0;
            task.id=TASK_IDLE_;
            task.preId=TASK_STEP_SINUNOID;
            ROS_INFO("task step sinunoid: finish");
        }
        break;
    case TASK_STEP_COMMAND:
        if(task.startFlag==0){


            if(task.setup_para==0)
            {
                ROS_INFO("task step command: start");
                switch(task.setup_para_count){
                case 0:
                    for(unsigned char i=0;i<12;i++){
                        samPID.P[i]=steping_samP[i];
                        samPID.D[i]=steping_samD[i];
                        samPID.I[i]=0;
                    }
                    sam_pid_pub.publish(samPID);
                    task.setup_para_count++;
                    break;
                case 1:
                    for(unsigned char i=0;i<12;i++){
                        samPID.P[i]=steping_samP[i];
                        samPID.D[i]=steping_samD[i];
                        samPID.I[i]=0;
                    }
                    sam_pid_pub.publish(samPID);
                    task.setup_para_count++;
                    break;
                case 2:
                    for(unsigned char i=0;i<12;i++){
                        samTorque.Torque[i]=steping_averageTorq[i];
                    }
                    sam_torque_pub.publish(samTorque);
                    task.setup_para_count++;
                    break;
                case 3:
                    for(unsigned char i=0;i<12;i++){
                        samTorque.Torque[i]=steping_averageTorq[i];
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


                task.startFlag=1;
                task.scene=0;
                currentScene.flag.finish=1;
                /*
                 * Your code begin from here
                 */
                sys_flag.sinunoid_motion=0;
                step_timer=0;
            }




        }
        else if(currentScene.flag.finish)
        {
            if(task.scene==0){
                task.scene=1;

                double beginpose[NUM_OF_SAM_];

                foot_left_position[0]=-foot_pos_x_offset;
                foot_left_position[1]=AcomY*sin(phiY_left)+foot_left_posY_offset;
                double b;
                b=foot_pos_z_amp*sin(phiZ_left);
                if(b<0)
                    b=0;
                foot_left_position[2]=-BODY_HEIGHT_LEFT+b;

                foot_right_position[0]=-foot_pos_x_offset;
                foot_right_position[1]=AcomY*sin(phiY_right)-foot_right_posY_offset;
                b=foot_pos_z_amp*sin(phiZ_right);
                if(b<0)
                    b=0;
                foot_right_position[2]=-BODY_HEIGHT_RIGHT+b;

                inverseKinematic(foot_left_mat,foot_left_position[0],foot_left_position[1],foot_left_position[2],calAngle);
                pose_step_sinunoid_init[1]=calAngle[5]*180/M_PI;
                pose_step_sinunoid_init[3]=-calAngle[4]*180/M_PI-ankle_pitch_offset;
                pose_step_sinunoid_init[5]=-calAngle[3]*180/M_PI;
                pose_step_sinunoid_init[7]=calAngle[2]*180/M_PI-body_tilt_offset;
                pose_step_sinunoid_init[9]=calAngle[1]*180/M_PI;
                pose_step_sinunoid_init[11]=calAngle[0]*180/M_PI;
                inverseKinematic(foot_right_mat,foot_right_position[0],foot_right_position[1],foot_right_position[2],calAngle);
                pose_step_sinunoid_init[0]=calAngle[5]*180/M_PI;
                pose_step_sinunoid_init[2]=calAngle[4]*180/M_PI+ankle_pitch_offset;
                pose_step_sinunoid_init[4]=calAngle[3]*180/M_PI;
                pose_step_sinunoid_init[6]=-calAngle[2]*180/M_PI+body_tilt_offset;
                pose_step_sinunoid_init[8]=calAngle[1]*180/M_PI;
                pose_step_sinunoid_init[10]=calAngle[0]*180/M_PI;

                currentScene.mapPos12ToDegree(currentControlledSamPos12,beginpose,NUM_OF_SAM_);
                currentScene.setUpMyScene(200,beginpose,pose_step_sinunoid_init);

            }else{
                unsigned char step_mode;
                switch(step_command){
                case STEP_CMD_WALK_FORWARD_LEFT:
                    if(step_count==0)
                    {
                        step_mode=STEP_MODE_FORWARD_IN;
                        leg_current_id=STEP_LEGID_LEFT;
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }
                    else if(step_count==step_number){
                        step_mode=STEP_MODE_FORWARD_OUT;
                        leg_current_id= (leg_current_id==STEP_LEGID_LEFT?STEP_LEGID_RIGHT:STEP_LEGID_LEFT);
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }
                    else if(step_count==(step_number+1)){
                        step_count=0;
                        task.finishFlag=1;
                    }else{
                        step_mode=STEP_MODE_FORWARD_NORMAL;
                        leg_current_id= (leg_current_id==STEP_LEGID_LEFT?STEP_LEGID_RIGHT:STEP_LEGID_LEFT);
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }
                    break;
                case STEP_CMD_WALK_FORWARD_RIGHT:
                    if(step_count==0)
                    {
                        step_mode=STEP_MODE_FORWARD_IN;
                        leg_current_id=STEP_LEGID_RIGHT;
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }
                    else if(step_count==step_number){

                        step_mode=STEP_MODE_FORWARD_OUT;
                        leg_current_id= (leg_current_id==STEP_LEGID_LEFT?STEP_LEGID_RIGHT:STEP_LEGID_LEFT);
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }
                    else if(step_count==(step_number+1)){
                        step_count=0;
                        task.finishFlag=1;
                    }else{
                        step_mode=STEP_MODE_FORWARD_NORMAL;
                        leg_current_id= (leg_current_id==STEP_LEGID_LEFT?STEP_LEGID_RIGHT:STEP_LEGID_LEFT);
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }
                    break;

                case STEP_CMD_WALK_ROTATE_LEFT:

                    if(step_count==step_number){
                        step_count=0;
                        task.finishFlag=1;
                    }else{
                        step_mode=STEP_MODE_ROTATE;
                        leg_current_id=STEP_LEGID_LEFT;
                        setUp_stepMotion(step_mode,leg_current_id,12);
                    }
                    break;
                case STEP_CMD_WALK_ROTATE_RIGHT:
                    if(step_count==step_number){
                        step_count=0;
                        task.finishFlag=1;
                    }else{
                        step_mode=STEP_MODE_ROTATE;
                        leg_current_id=STEP_LEGID_RIGHT;
                        setUp_stepMotion(step_mode,leg_current_id,12);
                    }

                    break;

                case STEP_CMD_WALK_INPLACE:
                    if(step_count==step_number){
                        step_count=0;
                        task.finishFlag=1;
                    }else{
                        step_mode=STEP_MODE_INPLACE;
                        leg_current_id= (leg_current_id==STEP_LEGID_LEFT?STEP_LEGID_RIGHT:STEP_LEGID_LEFT);
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }

                    break;
                case STEP_CMD_WALK_BACKWARD_LEFT:
                    if(step_count==0)
                    {
                        step_mode=STEP_MODE_BACKWARD_IN;
                        leg_current_id=STEP_LEGID_LEFT;
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }
                    else if(step_count==step_number){
                        step_mode=STEP_MODE_BACKWARD_OUT;
                        leg_current_id= (leg_current_id==STEP_LEGID_LEFT?STEP_LEGID_RIGHT:STEP_LEGID_LEFT);
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }
                    else if(step_count==(step_number+1)){
                        step_count=0;
                        task.finishFlag=1;
                    }else{
                        step_mode=STEP_MODE_BACKWARD_NORMAL;
                        leg_current_id= (leg_current_id==STEP_LEGID_LEFT?STEP_LEGID_RIGHT:STEP_LEGID_LEFT);
                        setUp_stepMotion(step_mode,leg_current_id,0);
                    }

                    break;
                case STEP_CMD_WALK_BACKWARD_RIGHT:
                    break;
                case STEP_CMD_WALK_SIDE_LEFT:
                    break;
                case STEP_CMD_WALK_SIDE_RIGHT:
                    break;
                default:
                    break;
                }
                if(task.finishFlag==0)
                    step_count++;
            }
            currentScene.flag.finish=0;

        }
        else if(task.finishFlag){

            task.finishFlag=0;
            task.startFlag=0;
            task.id=TASK_IDLE_;
            task.preId=TASK_STEP_COMMAND;
            ROS_INFO("task step command: finish");
        }
        break;
        break;
    default:
        break;
    }


    if(currentScene.flag.enable&&(sys_flag.sinunoid_motion==0)){
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
                    posSetPointDegree[i]=*(currentScene.beginPose+i);

                    double b=*(currentScene.beginPose+i);
                    double a=*(currentScene.endPose+i);
                    if(a==b){
                        delta_posDegree[i]=0;
                    }
                    else
                        delta_posDegree[i]=((double)a-(double)b)/(double)currentScene.numOfFrame;
                }


            }
            else if(currentScene.frame<currentScene.numOfFrame)
            {
                for(unsigned char i=0;i<NUM_OF_SAM_;i++)
                {
                    posSetPointDegree[i]+=delta_posDegree[i];
                }
            }
            else{
                currentScene.flag.enable=0;
                currentScene.flag.finish=1;
            }


        }
    }else if(sys_flag.sinunoid_motion)
    {
        step_timer+=SAMPLING_TIME_;
        if((step_timer>step_duration)&&(flag_enable_damping_left==0)&&((flag_enable_damping_right==0))){
            currentScene.flag.enable=0;
            currentScene.flag.finish=1;
            sys_flag.sinunoid_motion=0;
        }
        else if(step_timer<=step_duration){

            //            if(flag_step_firstTime==0)
            //            {
            //                flag_step_firstTime=1;
            //                flag_enable_step_in=1;
            //            }


            left_leg_height=foot_pos_z_amp*sin(omega*step_timer+phiZ_left)-foot_pos_z_offset;



            double comp_right=left_leg_height-0.07;
            if (comp_right<0){
                comp_right=0;
            }

            if(left_leg_height<0)
            {
                left_leg_height=0;
                //===========damping motion process=============
                if(step_left_status==STEP_UP)
                    step_left_status=STEP_DOWN;


                //===========X motion process=============
                flag_step_up_left=0;
                flag_Xmotion_resetTimer_left=0;
            }
            else{
                //===========damping motion process=============
                if(step_left_status==STEP_STANCE)
                {
                    step_left_status=STEP_UP;
                    flag_step_up_left=1;
                }
                flag_trigger_damping_left=0;
                flag_enable_damping_left=0;
                //===========X motion process=============

            }



            right_leg_height=foot_pos_z_amp*sin(omega*step_timer+phiZ_right)-foot_pos_z_offset;

            double comp_left=right_leg_height-0.07;
            if (comp_left<0){
                comp_left=0;
            }

            if(right_leg_height<0){
                right_leg_height=0;
                //===========damping motion process=============
                if(step_right_status==STEP_UP)
                    step_right_status=STEP_DOWN;


                //===========X motion process=============
                flag_step_up_right=0;
                flag_Xmotion_resetTimer_right=0;

            }
            else{
                //===========damping motion process=============
                if(step_right_status==STEP_STANCE)
                {
                    step_right_status=STEP_UP;
                     flag_step_up_right=1;
                }
                flag_trigger_damping_right=0;
                flag_enable_damping_right=0;

                //===========X motion process=============

            }




            //===========passive ankle process=============

            //            if((step_left_status==STEP_UP)){
            //                samPos12.SAMMode[0]=1;
            //                samPos12.SAMMode[1]=2;

            //            }else if((step_right_status==STEP_UP)){
            //                samPos12.SAMMode[0]=2;
            //                samPos12.SAMMode[1]=1;
            //            }
            //            else{
            //                samPos12.SAMMode[0]=1;
            //                samPos12.SAMMode[1]=1;
            //            }

            //            if(step_right_status==STEP_STANCE){
            //                samPos12.SAMMode[0]=1;
            //                samPos12.SAMMode[1]=1;

            //            }else{
            //                samPos12.SAMMode[0]=2;
            ////                samPos12.SAMMode[1]=2;
            //            }

            if(flag_step_up_left)
            {
                if(flag_Xmotion_resetTimer_left==0){
                    flag_Xmotion_resetTimer_left=1;
                    step_Xmotion_timer=0;
                    rotate_step_count++;
                }

                if(step_Xmotion_timer>periodT_X/2){
                    flag_step_up_left=0;
//                    flag_Xmotion_resetTimer_left=0;
                }


                if(flag_enable_step_in)
                {
                    double step_in_func=AcomX*(1-step_Xmotion_timer*omega_X/M_PI);
                    foot_right_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_right)-step_in_func-foot_pos_x_offset;
                    foot_left_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_left)+step_in_func-foot_pos_x_offset;
                }else if(flag_enable_step_out){
                    double step_out_func=AcomX*step_Xmotion_timer*omega_X/M_PI;
                    foot_right_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_right)+step_out_func-foot_pos_x_offset;
                    foot_left_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_left)-step_out_func-foot_pos_x_offset;
                }
                else{
                    foot_right_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_right)-foot_pos_x_offset;
                    foot_left_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_left)-foot_pos_x_offset;
                }
                step_Xmotion_timer+=SAMPLING_TIME_;

                if(flag_enable_step_yaw){
                    if(rotate_step_count==1){
                        foot_left_yaw=Ayaw_left*sin(omega_yaw*step_Xmotion_timer+phiYaw);
                        foot_right_yaw=Ayaw_right*sin(omega_yaw*step_Xmotion_timer+phiYaw);
                    }else if(rotate_step_count==2) {
                        foot_left_yaw=Ayaw_left*sin(omega_yaw*step_Xmotion_timer+phiYaw+M_PI/2);
                        foot_right_yaw=Ayaw_right*sin(omega_yaw*step_Xmotion_timer+phiYaw+M_PI/2);
                    }
                    double c_y=cos(foot_left_yaw);
                    double s_y=sin(foot_left_yaw);
                    foot_left_mat[INDEX(1,1)]=c_y;
                    foot_left_mat[INDEX(1,2)]=s_y;
                    foot_left_mat[INDEX(2,1)]=-s_y;
                    foot_left_mat[INDEX(2,2)]=c_y;
                    c_y=cos(foot_right_yaw);
                    s_y=-sin(foot_right_yaw);
                    foot_right_mat[INDEX(1,1)]=c_y;
                    foot_right_mat[INDEX(1,2)]=s_y;
                    foot_right_mat[INDEX(2,1)]=-s_y;
                    foot_right_mat[INDEX(2,2)]=c_y;
                }

            }else if(flag_step_up_right){
                if(flag_Xmotion_resetTimer_right==0){
                    flag_Xmotion_resetTimer_right=1;
                    step_Xmotion_timer=0;
                    rotate_step_count++;
                }

                if(step_Xmotion_timer>periodT_X/2){
                    flag_step_up_right=0;
                }

                if(flag_enable_step_in)
                {
                    double step_in_func=AcomX*(1-step_Xmotion_timer*omega_X/M_PI);
                    foot_right_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_right)+step_in_func-foot_pos_x_offset;
                    foot_left_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_left)-step_in_func-foot_pos_x_offset;
                }else if(flag_enable_step_out){
                    double step_out_func=AcomX*step_Xmotion_timer*omega_X/M_PI;
                    foot_right_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_right)-step_out_func-foot_pos_x_offset;
                    foot_left_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_left)+step_out_func-foot_pos_x_offset;
                }else{
                    foot_right_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_right)-foot_pos_x_offset;
                    foot_left_position[0]=AcomX*sin(omega_X*(step_Xmotion_timer)+phiX_left)-foot_pos_x_offset;
                }
                step_Xmotion_timer+=SAMPLING_TIME_;

                if(flag_enable_step_yaw){

                    if(rotate_step_count==1){
                        foot_left_yaw=Ayaw_left*sin(omega_yaw*step_Xmotion_timer+phiYaw);
                        foot_right_yaw=Ayaw_right*sin(omega_yaw*step_Xmotion_timer+phiYaw);
                    }else if(rotate_step_count==2) {
                        foot_left_yaw=Ayaw_left*sin(omega_yaw*step_Xmotion_timer+phiYaw+M_PI/2);
                        foot_right_yaw=Ayaw_right*sin(omega_yaw*step_Xmotion_timer+phiYaw+M_PI/2);
                    }

                    double c_y=cos(foot_left_yaw);
                    double s_y=sin(foot_left_yaw);
                    //                    foot_left_mat[INDEX(1,1)]=c_y;
                    //                    foot_left_mat[INDEX(1,2)]=s_y*c_roll;

                    //                    foot_left_mat[INDEX(1,3)]=-s_y*s_roll;
                    //                    foot_left_mat[INDEX(2,1)]=-s_y;
                    //                    foot_left_mat[INDEX(2,2)]=c_y*c_roll;

                    //                    foot_left_mat[INDEX(2,3)]=-c_y*s_roll;

                    //                    foot_left_mat[INDEX(3,2)]=s_roll;
                    //                    foot_left_mat[INDEX(3,3)]=c_roll;

                    foot_left_mat[INDEX(1,1)]=c_y;
                    foot_left_mat[INDEX(1,2)]=s_y;
                    foot_left_mat[INDEX(2,1)]=-s_y;
                    foot_left_mat[INDEX(2,2)]=c_y;
                    c_y=cos(foot_right_yaw);
                    s_y=-sin(foot_right_yaw);
                    foot_right_mat[INDEX(1,1)]=c_y;
                    foot_right_mat[INDEX(1,2)]=s_y;
                    foot_right_mat[INDEX(2,1)]=-s_y;
                    foot_right_mat[INDEX(2,2)]=c_y;
                }
            }

            double c_y=0;
            double s_y=0;
            //            foot_left_mat[INDEX(1,1)]=c_y;
            //            foot_left_mat[INDEX(1,2)]=s_y*c_roll;

            //            foot_left_mat[INDEX(1,3)]=-s_y*s_roll;
            //            foot_left_mat[INDEX(2,1)]=-s_y;
            //            foot_left_mat[INDEX(2,2)]=c_y*c_roll;

            //            foot_left_mat[INDEX(2,3)]=-c_y*s_roll;

            //            foot_left_mat[INDEX(3,2)]=s_roll;
            //            foot_left_mat[INDEX(3,3)]=c_roll;



            //            foot_left_mat[INDEX(2,2)]=c_roll;
            //            foot_left_mat[INDEX(2,3)]=-s_roll;
            //            foot_left_mat[INDEX(3,2)]=s_roll;
            //            foot_left_mat[INDEX(3,3)]=c_roll;

            foot_left_position[1]=(AcomY*sin(omega*step_timer+phiY_left)+comp_left*0.1)+foot_left_posY_offset;
            foot_right_position[1]=(AcomY*sin(omega*step_timer+phiY_right)-comp_right*0.1)-foot_right_posY_offset;//0.2
            //rotation process

            //            plotMsg.CN7=foot_left_yaw;
            //            plotMsg.CN8=foot_right_yaw;
            //            plotMsg.CN9=pid_body_pitch.output;
            //            plotMsg.CN10=pid_body_pitch.fb;
            //            plotMsg.CN11=pid_body_pitch.set_point;
            //            //            plotMsg.CN12=pid_joint[9].output;
            //            plot_pub.publish(plotMsg);
        }

        if(step_left_status==STEP_DOWN){
            if(flag_trigger_damping_left==0)
            {
                flag_trigger_damping_left=1;
                //                        step_damping_timer_t0=step_timer;
                flag_enable_damping_left=1;
                step_damping_left_timer_t=0;
            }

            if(flag_enable_damping_left){

                damping_left=FOOT_POS_Z_DAMPING*sin(omega_damping*(step_damping_left_timer_t));
                if(damping_left<0)
                {
                    flag_enable_damping_left=0;
                    damping_left=0;
                    step_left_status=STEP_STANCE;

                }
                step_damping_left_timer_t+=0.01;;
            }
        }

        if(step_right_status==STEP_DOWN){
            if(flag_trigger_damping_right==0)
            {
                flag_trigger_damping_right=1;
                //                        step_damping_timer_t0=step_timer;
                step_damping_right_timer_t=0;
                flag_enable_damping_right=1;
            }

            if(flag_enable_damping_right){

                damping_right=FOOT_POS_Z_DAMPING*sin(omega_damping*(step_damping_right_timer_t));
                if(damping_right<0)
                {
                    flag_enable_damping_right=0;
                    damping_right=0;
                    step_right_status=STEP_STANCE;
                }
                step_damping_right_timer_t+=0.01;
            }
        }

        foot_left_position[2]=-BODY_HEIGHT_LEFT+left_leg_height-right_leg_height*0.04+damping_left+damping_right;
        foot_right_position[2]=-BODY_HEIGHT_RIGHT+right_leg_height-left_leg_height*0.04+damping_right+damping_left;//0.06

        inverseKinematic(foot_left_mat,foot_left_position[0],foot_left_position[1],foot_left_position[2],calAngle);

        posSetPointDegree[1]=calAngle[5]*180/M_PI;
        posSetPointDegree[3]=-calAngle[4]*180/M_PI-ankle_pitch_offset;
        posSetPointDegree[5]=-calAngle[3]*180/M_PI;
        posSetPointDegree[7]=calAngle[2]*180/M_PI-body_tilt_offset;
        posSetPointDegree[9]=calAngle[1]*180/M_PI;
        posSetPointDegree[11]=calAngle[0]*180/M_PI;



        inverseKinematic(foot_right_mat,foot_right_position[0],foot_right_position[1],foot_right_position[2],calAngle);

        posSetPointDegree[0]=calAngle[5]*180/M_PI;
        posSetPointDegree[2]=calAngle[4]*180/M_PI+ankle_pitch_offset;
        posSetPointDegree[4]=calAngle[3]*180/M_PI;
        posSetPointDegree[6]=-calAngle[2]*180/M_PI+body_tilt_offset;
        posSetPointDegree[8]=calAngle[1]*180/M_PI;
        posSetPointDegree[10]=calAngle[0]*180/M_PI;
    }

}

void Scene_class::setBeginPose(double *value)
{
    beginPose = value;
}

void Scene_class::setEndPose(double *value)
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

void Scene_class::setUpMyScene(unsigned int fr, double *beginpose, double *endpose)
{
    //    mapSoftToHardPose(beginpose,this->buf_beginPose,NUM_OF_SAM_);
    //    mapSoftToHardPose(endpose,this->buf_endPose,NUM_OF_SAM_);
    //    this->setBeginPose(buf_beginPose);
    //    this->setEndPose(buf_endPose);

    this->setNumOfFrame(fr);

    this->frame=0;
    this->flag.start=0;
    this->flag.finish=0;
    this->flag.enable=1;
    this->flag.delay=0;

    this->setBeginPose(beginpose);
    this->setEndPose(endpose);

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
        *(softwarePose+i)=(int)*(hardwarePose+i)-(int)samPos12_hardware[i];
    }
}

void Scene_class::mapPos12ToDegree(unsigned int *hardwarePose, double *degreePose, unsigned char size)
{
    for(unsigned char i=0;i<size;i++)
    {
        *(degreePose+i)=((double)*(hardwarePose+i)-(double)samPos12_hardware[i])*pos12bitTodegree;
    }
}

//void Scene_class::mapDegreeToPos12(double *degreePose, unsigned int *hardwarePose, unsigned char size)
//{
//    for(unsigned char i=0;i<size;i++)
//    {
//        *(hardwarePose+i)=(unsigned int)((*(degreePose+i))*degreeToPose12+(double)samPos12_hardware[i]);
//    }
//}

void sub_function_setPIDJoints(const uxa_motion_control::uxaSetPIDJointMsg::ConstPtr& msg){
    unsigned char index= msg->pidJointId;
    ROS_INFO("set pid joint %d",index);
    switch(msg->pidEnable){
    case 1:
        cout<<"pid "<< (int)index<<" enable"<<endl;
        sys_flag.controller_joint[index]=1;
        break;
    case 2:
        pid_joint[index].KP=msg->Pvalue;
        pid_joint[index].KI=msg->Ivalue;
        pid_joint[index].KD=msg->Dvalue;
        cout<< "KP = "<<pid_joint[index].KP <<"| KI = "<<pid_joint[index].KI<<"| KD = "<<pid_joint[index].KD<<endl;
        break;
    case 3:
        pid_joint[index].set_point=msg->setPoint;
        cout<< "setPoint = "<<pid_joint[index].set_point<<endl;
        break;
    case 4:
        cout<<"pid  enable all"<<endl;
        for(unsigned char i=0; i<12;i++){
            sys_flag.controller_joint[i]=1;
        }
        PID_reset();
        break;

    case 5:
        cout<<"pid  disable all"<<endl;
        for(unsigned char i=0; i<12;i++){
            sys_flag.controller_joint[i]=0;
        }
        PID_reset();
        break;
    case 6:
        cout<<"set all pid:"<<endl;
        for(unsigned char i=0; i<12;i++){
            pid_joint[i].KP=msg->Pvalue;
            pid_joint[i].KI=msg->Ivalue;
            pid_joint[i].KD=msg->Dvalue;
        }

        cout<< "KP = "<<msg->Pvalue <<"| KI = "<<msg->Ivalue<<"| KD = "<<msg->Dvalue<<endl;
        break;

    default:
        cout<<"pid "<< (int)index<<" disable"<<endl;
        sys_flag.controller_joint[index]=0;
        break;
    }



}


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

    case PID_IMPEDANCE_RIGHT_:
        pid_impedance_right.KP=msg->Pvalue;
        pid_impedance_right.KI=msg->Ivalue;
        pid_impedance_right.KD=msg->Dvalue;
        ROS_INFO("PID_IMPEDANCE_RIGHT KP=%f| KI=%f| KD=%f",pid_impedance_right.KP,pid_impedance_right.KI,pid_impedance_right.KD);
        break;
    case SETPOINT_IMPEDANCE_RIGHT_:
        pid_impedance_right.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_IMPEDANCE_RIGHT setpoint=%f",pid_impedance_right.set_point);
        break;

    case PID_ANKLE_LEFT_ROLL_:
        pid_ankle_left_roll.KP=msg->Pvalue;
        pid_ankle_left_roll.KI=msg->Ivalue;
        pid_ankle_left_roll.KD=msg->Dvalue;
        ROS_INFO("PID_ANKLE_LEFT_ROLL_ KP=%f| KI=%f| KD=%f",pid_ankle_left_roll.KP,pid_ankle_left_roll.KI,pid_ankle_left_roll.KD);
        break;
    case SETPOINT_ANKLE_LEFT_ROLL_:
        pid_ankle_left_roll.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_ANKLE_LEFT_ setpoint=%f",pid_ankle_left_roll.set_point);
        break;

    case PID_ANKLE_RIGHT_ROLL_:
        pid_ankle_right_roll.KP=msg->Pvalue;
        pid_ankle_right_roll.KI=msg->Ivalue;
        pid_ankle_right_roll.KD=msg->Dvalue;
        ROS_INFO("PID_ANKLE_RIGHT_ROLL KP=%f| KI=%f| KD=%f",pid_ankle_right_roll.KP,pid_ankle_right_roll.KI,pid_ankle_right_roll.KD);
        break;

    case SETPOINT_ANKLE_RIGHT_ROLL_:
        pid_ankle_right_roll.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_ANKLE_RIGHT_ROLL setpoint=%f",pid_ankle_right_roll.set_point);
        break;

    case PID_ANKLE_LEFT_PITCH_:
        pid_ankle_left_pitch.KP=msg->Pvalue;
        pid_ankle_left_pitch.KI=msg->Ivalue;
        pid_ankle_left_pitch.KD=msg->Dvalue;
        ROS_INFO("PID_ANKLE_LEFT_PITCH_ KP=%f| KI=%f| KD=%f",pid_ankle_left_pitch.KP,pid_ankle_left_pitch.KI,pid_ankle_left_pitch.KD);
        break;
    case SETPOINT_ANKLE_LEFT_PITCH_:
        pid_ankle_left_pitch.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_ANKLE_LEFT_ setpoint=%f",pid_ankle_left_pitch.set_point);
        break;

    case PID_ANKLE_RIGHT_PITCH_:
        pid_ankle_right_pitch.KP=msg->Pvalue;
        pid_ankle_right_pitch.KI=msg->Ivalue;
        pid_ankle_right_pitch.KD=msg->Dvalue;
        ROS_INFO("PID_ANKLE_RIGHT_PITCH KP=%f| KI=%f| KD=%f",pid_ankle_right_pitch.KP,pid_ankle_right_pitch.KI,pid_ankle_right_pitch.KD);
        break;

    case SETPOINT_ANKLE_RIGHT_PITCH_:
        pid_ankle_right_pitch.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_ANKLE_RIGHT_PITCH setpoint=%f",pid_ankle_right_pitch.set_point);
        break;

    case PID_FOOT_PLACE_:
        pid_foot_place.KP=msg->Pvalue;
        pid_foot_place.KI=msg->Ivalue;
        pid_foot_place.KD=msg->Dvalue;
        ROS_INFO("PID_FOOT_PLACE_ KP=%f| KI=%f| KD=%f",pid_foot_place.KP,pid_foot_place.KI,pid_foot_place.KD);
        break;
    case SETPOINT_FOOT_PLACE_:
        pid_foot_place.set_point=msg->setPoint;
        ROS_INFO("SETPOINT_FOOT_PLACE_ setpoint=%f",pid_foot_place.set_point);
        break;


    default:
        //        ROS_INFO("PID_SET_BODY_PITCH_ KP=%f| KI=%f| KD=%f",pid_body_pitch.KP,pid_body_pitch.KI,pid_body_pitch.KD);
        //        ROS_INFO("SETPOINT_SET_BODY_PITCH_ setpoint=%f",pid_body_pitch.set_point);

        //        ROS_INFO("PID_SET_BODY_ROLL_ KP=%f| KI=%f| KD=%f",pid_body_roll_0_1.KP,pid_body_roll_0_1.KI,pid_body_roll_0_1.KD);
        //        ROS_INFO("SETPOINT_SET_BODY_ROLL_ setpoint=%f",pid_body_roll_0_1.set_point);

        ROS_INFO("PID_ANKLE_LEFT_ROLL_ KP=%f| KI=%f| KD=%f| Setpoint=%f",pid_ankle_left_roll.KP,pid_ankle_left_roll.KI,pid_ankle_left_roll.KD,pid_ankle_left_roll.set_point);
        ROS_INFO("PID_ANKLE_RIGHT_ROLL KP=%f| KI=%f| KD=%f| Setpoint=%f",pid_ankle_right_roll.KP,pid_ankle_right_roll.KI,pid_ankle_right_roll.KD,pid_ankle_right_roll.set_point);
        ROS_INFO("PID_FOOT_PLACE_ KP=%f| KI=%f| KD=%f| Setpoint=%f",pid_foot_place.KP,pid_foot_place.KI,pid_foot_place.KD,pid_foot_place.set_point);
        ROS_INFO("PID_ANKLE_LEFT_pitch_ KP=%f| KI=%f| KD=%f| Setpoint=%f",pid_ankle_left_pitch.KP,pid_ankle_left_pitch.KI,pid_ankle_left_pitch.KD,pid_ankle_left_pitch.set_point);
        ROS_INFO("PID_ANKLE_RIGHT_pitch KP=%f| KI=%f| KD=%f| Setpoint=%f",pid_ankle_right_pitch.KP,pid_ankle_right_pitch.KI,pid_ankle_right_pitch.KD,pid_ankle_right_pitch.set_point);

        break;
    }
}

void sub_function_walking(const uxa_motion_control::uxaMotionCommandMsg::ConstPtr& msg){

    step_number=msg->numOfStep;
    if(step_number==0)
        step_number=1;
    switch (msg->motionMode) {
    case STEP_CMD_WALK_FORWARD_LEFT:
        step_command=STEP_CMD_WALK_FORWARD_LEFT;
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STEP_SINUNOID|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_COMMAND){
            ROS_INFO("STEP_CMD_WALK_FORWARD_LEFT %d", msg->motionMode);
            task.id=TASK_STEP_COMMAND;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;
    case STEP_CMD_WALK_FORWARD_RIGHT:
        step_command=STEP_CMD_WALK_FORWARD_RIGHT;
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STEP_SINUNOID|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_COMMAND){
            ROS_INFO("STEP_CMD_WALK_FORWARD_RIGHT %d", msg->motionMode);
            task.id=TASK_STEP_COMMAND;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;
    case STEP_CMD_WALK_ROTATE_LEFT:
        step_command=STEP_CMD_WALK_ROTATE_LEFT;
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STEP_SINUNOID|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_COMMAND){
            ROS_INFO("STEP_CMD_WALK_ROTATE_LEFT %d", msg->motionMode);
            task.id=TASK_STEP_COMMAND;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;

    case STEP_CMD_WALK_ROTATE_RIGHT:
        step_command=STEP_CMD_WALK_ROTATE_RIGHT;
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STEP_SINUNOID|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_COMMAND){
            ROS_INFO("STEP_CMD_WALK_ROTATE_RIGHT %d", msg->motionMode);
            task.id=TASK_STEP_COMMAND;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;

    case STEP_CMD_WALK_INPLACE:
        step_command=STEP_CMD_WALK_INPLACE;
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STEP_SINUNOID|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_COMMAND){
            ROS_INFO("STEP_CMD_WALK_INPLACE %d", msg->motionMode);
            task.id=TASK_STEP_COMMAND;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;

    case STEP_CMD_WALK_BACKWARD_LEFT:
        step_command=STEP_CMD_WALK_BACKWARD_LEFT;
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STEP_SINUNOID|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_COMMAND){
            ROS_INFO("STEP_CMD_WALK_BACKWARD_LEFT %d", msg->motionMode);
            task.id=TASK_STEP_COMMAND;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;

    default:
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
        task.id=TASK_SITDOWN_F_INIT;
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
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_TEST|task.preId==TASK_STEP_SINUNOID|task.preId==TASK_STEP_COMMAND){
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
    case MOTION_SET_SINUNOID_F_STANDINGINIT:
        if(task.preId==TASK_STANDINGINIT_F_STANDING|task.preId==TASK_STEP_SINUNOID|task.preId==TASK_STANDINGINIT_F_STEP|task.preId==TASK_STEP_COMMAND){
            ROS_INFO("MOTION_SET_SINUNOID_F_STANDINGINIT %d", msg->command);
            task.id=TASK_STEP_SINUNOID;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }

        //==========================
        task.id=TASK_STEP_SINUNOID;
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

    case MOTION_ENALBLE_CONTROL_ANKLE_LEFT_ROLL:
        ROS_INFO("MOTION_ENALBLE_CONTROL_ANKLE_LEFT_ROLL: %d", msg->command);
        sys_flag.controller_ankle_left_roll=1;
        break;
    case MOTION_DISABLE_CONTROL_ANKLE_LEFT_ROLL:
        ROS_INFO("MOTION_DISABLE_CONTROL_ANKLE_LEFT_ROLL: %d", msg->command);
        sys_flag.controller_ankle_left_roll=0;
        break;

    case MOTION_ENALBLE_CONTROL_ANKLE_RIGHT_ROLL:
        ROS_INFO("MOTION_ENALBLE_CONTROL_ANKLE_RIGHT_ROLL: %d", msg->command);
        sys_flag.controller_ankle_right_roll=1;
        break;
    case MOTION_DISABLE_CONTROL_ANKLE_RIGHT_ROLL:
        ROS_INFO("MOTION_DISABLE_CONTROL_ANKLE_RIGHT_ROLL: %d", msg->command);
        sys_flag.controller_ankle_right_roll=0;
        break;

    case MOTION_ENALBLE_CONTROL_ANKLE_LEFT_PITCH:
        ROS_INFO("MOTION_ENALBLE_CONTROL_ANKLE_LEFT_pitch: %d", msg->command);
        sys_flag.controller_ankle_left_pitch=1;
        break;
    case MOTION_DISABLE_CONTROL_ANKLE_LEFT_PITCH:
        ROS_INFO("MOTION_DISABLE_CONTROL_ANKLE_LEFT_pitch: %d", msg->command);
        sys_flag.controller_ankle_left_pitch=0;
        break;

    case MOTION_ENALBLE_CONTROL_ANKLE_RIGHT_PITCH:
        ROS_INFO("MOTION_ENALBLE_CONTROL_ANKLE_RIGHT_pitch: %d", msg->command);
        sys_flag.controller_ankle_right_pitch=1;
        break;
    case MOTION_DISABLE_CONTROL_ANKLE_RIGHT_PITCH:
        ROS_INFO("MOTION_DISABLE_CONTROL_ANKLE_RIGHT_pitch: %d", msg->command);
        sys_flag.controller_ankle_right_pitch=0;
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
    imu.pitch_rate=msg->body_pitch_rate;
    imu.roll_rate=msg->body_roll_rate;
    imu.yaw_rate=msg->body_yaw_rate;

    rightZMP.amp_filtered=rightZMP.amp_filtered*0.9+0.1*rightZMP.amp;
    rightZMP.delta_amp=rightZMP.amp_filtered-rightZMP.pre_amp;
    rightZMP.pre_amp=rightZMP.amp_filtered;

    leftZMP.amp_filtered=leftZMP.amp_filtered*0.9+0.1*leftZMP.amp;
    leftZMP.delta_amp=leftZMP.amp_filtered-leftZMP.pre_amp;
    leftZMP.pre_amp=leftZMP.amp_filtered;

    if((abs(rightZMP.delta_amp)<1.2)&&(abs(leftZMP.delta_amp<1.2)))
        stable_pose_flag=1;
    else
        stable_pose_flag=0;

    delta_amp=abs(leftZMP.amp-rightZMP.amp);
    double total_amp=leftZMP.amp+rightZMP.amp;
    if(total_amp>0){
        COMpos=(leftZMP.amp* leftFootPos[0]+rightZMP.amp*rightFootPos[0])/total_amp;
    }
}

void sam_callback(const uxa_motion_control::SAMJointStateMsg::ConstPtr& msg){
    //    ROS_INFO("sam data: %d",msg->SAMPos12[0]);
    sys_flag.feedback_sam_available=1;
    for(unsigned char i=0;i<NUM_OF_SAM_;i++)
    {
        currentSAMPosDegree[i]=msg->SAMPosDegree[i];
        //        currentSAMPosDegree[i]=(currentSamPos12[i]-samPos12_hardware[i])*pos12bitTodegree;
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

void PID_control_init(){

    pid_body_pitch.sampling_time=SAMPLING_TIME_;
    pid_body_pitch.reset_parameters();
    pid_body_pitch.KP=0;
    pid_body_pitch.KI=11;
    pid_body_pitch.KD=0;
    pid_body_pitch.I_limit=200;
    pid_body_pitch.D_limit=100;
    pid_body_pitch.output_limit=400;
    pid_body_pitch.set_point=-BODY_PITCH_SETPOINT_STANDING;

    //    pid_impedance_right.sampling_time=SAMPLING_TIME_;
    //    pid_impedance_right.reset_parameters();
    //    pid_impedance_right.KP=0.005;
    //    pid_impedance_right.KI=0;
    //    pid_impedance_right.KD=0;
    //    pid_impedance_right.I_limit=0.1;
    //    pid_impedance_right.D_limit=0;
    //    pid_impedance_right.output_limit=1;
    //    pid_impedance_right.set_point=100;

    //    pid_impedance_left.sampling_time=SAMPLING_TIME_;
    //    pid_impedance_left.reset_parameters();
    //    pid_impedance_left.KP=0.005;
    //    pid_impedance_left.KI=0;
    //    pid_impedance_left.KD=0;
    //    pid_impedance_left.I_limit=0.1;
    //    pid_impedance_left.D_limit=0;
    //    pid_impedance_left.output_limit=1;
    //    pid_impedance_left.set_point=100;



    //    pid_ankle_right_pitch.sampling_time=SAMPLING_TIME_;
    //    pid_ankle_right_pitch.reset_parameters();
    //    pid_ankle_right_pitch.KP=0;
    //    pid_ankle_right_pitch.KI=1;
    //    pid_ankle_right_pitch.KD=0;
    //    pid_ankle_right_pitch.I_limit=15;
    //    pid_ankle_right_pitch.D_limit=5;
    //    pid_ankle_right_pitch.output_limit=20;
    //    pid_ankle_right_pitch.set_point=ankle_right_pitch_setpoint;


    //    pid_ankle_right_roll.sampling_time=SAMPLING_TIME_;
    //    pid_ankle_right_roll.reset_parameters();
    //    pid_ankle_right_roll.KP=0;
    //    pid_ankle_right_roll.KI=1;
    //    pid_ankle_right_roll.KD=0;
    //    pid_ankle_right_roll.I_limit=15;
    //    pid_ankle_right_roll.D_limit=5;
    //    pid_ankle_right_roll.output_limit=20;
    //    pid_ankle_right_roll.set_point=ankle_right_roll_setpoint;

    //    pid_ankle_left_pitch.sampling_time=SAMPLING_TIME_;
    //    pid_ankle_left_pitch.reset_parameters();
    //    pid_ankle_left_pitch.KP=0;
    //    pid_ankle_left_pitch.KI=1;
    //    pid_ankle_left_pitch.KD=0;
    //    pid_ankle_left_pitch.I_limit=15;
    //    pid_ankle_left_pitch.D_limit=5;
    //    pid_ankle_left_pitch.output_limit=20;
    //    pid_ankle_left_pitch.set_point=ankle_left_pitch_setpoint;


    //    pid_ankle_left_roll.sampling_time=SAMPLING_TIME_;
    //    pid_ankle_left_roll.reset_parameters();
    //    pid_ankle_left_roll.KP=0;
    //    pid_ankle_left_roll.KI=1;
    //    pid_ankle_left_roll.KD=0;
    //    pid_ankle_left_roll.I_limit=15;
    //    pid_ankle_left_roll.D_limit=5;
    //    pid_ankle_left_roll.output_limit=20;
    //    pid_ankle_left_roll.set_point=ankle_left_roll_setpoint;

    //    pid_foot_place.sampling_time=SAMPLING_TIME_;
    //    pid_foot_place.reset_parameters();
    //    pid_foot_place.KP=0;
    //    pid_foot_place.KI=1000;//10000;
    //    pid_foot_place.KD=0;
    //    pid_foot_place.I_limit=15;//70;
    //    pid_foot_place.D_limit=5;//50;
    //    pid_foot_place.output_limit=30;//100;
    //    pid_foot_place.set_point=foot_place_setpoint;


    for(unsigned char i=0;i<12;i++)
    {
        pid_joint[i].sampling_time=SAMPLING_TIME_;
        pid_joint[i].reset_parameters();
        pid_joint[i].KP=5;
        pid_joint[i].KI=80;
        pid_joint[i].KD=0.05;
        pid_joint[i].I_limit=200;
        pid_joint[i].D_limit=100;
        pid_joint[i].output_limit=400;
        pid_joint[i].set_point=0;
    }
}
void PID_reset()
{
    pid_body_pitch.reset_parameters();
    //    pid_impedace_ouputIntegrate_right=0;
    //    pid_impedace_ouputIntegrate_left=0;

    //    pid_ankle_left_roll.reset_parameters();
    //    pid_ankle_right_roll.reset_parameters();
    //    pid_ankle_left_pitch.reset_parameters();
    //    pid_ankle_right_pitch.reset_parameters();
    //    pid_foot_place.reset_parameters();
    //    pid_impedance_left.reset_parameters();
    //    pid_impedance_right.reset_parameters();
    for(unsigned char i=0;i<12;i++)
    {
        pid_joint[i].reset_parameters();
    }
    for(unsigned char i=0;i<NUM_OF_SAM_;i++)
    {
        controller_output[i]=0;
    }
}



void PID_joint_enable(unsigned char stt){
    if(stt){
        for(unsigned char i=0;i<12;i++)
        {
            sys_flag.controller_joint[i]=1;
        }
    }else{
        for(unsigned char i=0;i<12;i++)
        {
            sys_flag.controller_joint[i]=0;
        }
    }
}
