#include <iostream>
#include <ros/ros.h>
using namespace std;
#include "uxa_motion_control/main_uxa_motion_control.h"

#include "uxa_motion_control/motionCmdMsg.h"
#include "uxa_motion_control/dataSensorMsg.h"
#include "uxa_motion_control/cmdSensorMsg.h"

#include "uxa_motion_control/SAMcmdMsg.h"
#include "uxa_motion_control/SAMJointPos12Msg.h"
#include "uxa_motion_control/SAMJointStateMsg.h"

#include "uxa_motion_control/displayCmdMsg.h"

ros::Publisher sensor_cmd_pub;
ros::Publisher sam_cmd_pub;
ros::Publisher display_cmd_pub;

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
        sensor_command.command=SENSOR_SEND_FILTERED_50HZ;
        sensor_cmd_pub.publish(sensor_command);
        break;

    case MOTION_DISABLE_FB_SAM:
        ROS_INFO("MOTION_DISABLE_FB_SAM : %d", msg->command);
        sam_command.command=SAM_FB_TURNOFF;
        sam_cmd_pub.publish(sam_command);
        break;
    case MOTION_ENABLE_FB_SAM:
        ROS_INFO("MOTION_ENABLE_FB_SAM: %d", msg->command);
        sam_command.command=SAM_FB_LOWERLINK_POS12_50HZ;
        sam_cmd_pub.publish(sam_command);
        break;
    case MOTION_ENABLE_FB_ALL:
        sam_command.command=SAM_FB_LOWERLINK_POS12_50HZ;
        sam_cmd_pub.publish(sam_command);
        sensor_command.command=SENSOR_SEND_FILTERED_50HZ;
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
        break;
    case MOTION_STANDINGINIT_F_STANDING:
        if(task.preId==TASK_STANDING_F_SITDOWN){
            ROS_INFO("MOTION_STANDING_INIT %d", msg->command);
            task.id=TASK_STANDINGINIT_F_STANDING;
        }else{
            ROS_ERROR("error task order, pre task id: %d", task.preId);
        }
        break;
    case MOTION_STANDING_F_STADINGINIT:
        if(task.preId==TASK_STANDINGINIT_F_STANDING){
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
    ROS_INFO("sensor data: %d",msg->zmp_P0 );
    sys_flag.feedback_sensor_available=1;
    leftZMP.filterForceSensor[0]=msg->zmp_P0;
    leftZMP.filterForceSensor[1]=msg->zmp_P1;
    leftZMP.filterForceSensor[2]=msg->zmp_P2;
    leftZMP.filterForceSensor[3]=msg->zmp_P3;

    rightZMP.filterForceSensor[0]=msg->zmp_P4;
    rightZMP.filterForceSensor[1]=msg->zmp_P5;
    rightZMP.filterForceSensor[2]=msg->zmp_P6;
    rightZMP.filterForceSensor[3]=msg->zmp_P7;

    imu.roll=msg->body_roll;
    imu.pitch=msg->body_pitch;
    imu.yaw=msg->body_yaw;
}

void sam_callback(const uxa_motion_control::SAMJointStateMsg::ConstPtr& msg){
    //    ROS_INFO("sam data: %d",msg->SAMPos12[0]);
    sys_flag.feedback_sam_available=1;
    for(unsigned char i=0;i<25;i++)
    {
        currentSamPos12[i]=msg->SAMPos12[i];
        currentSamAvail[i]=msg->SAMPos12Avail[i];
    }
}

uxa_motion_control::SAMJointPos12Msg samPos12;
ros::Publisher sam_pos12_pub;

void task_handle();
int main(int argc, char **argv){
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE_1000Hz_);

    //    ros::Publisher sensor_cmd_pub =  n.advertise<uxa_motion_control::cmdSensorMsg>("sensor_sub",200);
    sensor_cmd_pub =  n.advertise<uxa_motion_control::cmdSensorMsg>("sensor_sub",200);
    ros::Subscriber sensor_data_sub =  n.subscribe<uxa_motion_control::dataSensorMsg>("sensor_pub",1000,sensor_callback);

    sam_cmd_pub =  n.advertise<uxa_motion_control::SAMcmdMsg>("sam_cmd_sub",200);
    display_cmd_pub =  n.advertise<uxa_motion_control::displayCmdMsg>("display_cmd",200);

    sam_pos12_pub=n.advertise<uxa_motion_control::SAMJointPos12Msg>("sam_pos12_sub",500);
    ros::Subscriber sam_joint_state_sub =  n.subscribe<uxa_motion_control::SAMJointStateMsg>("sam_pub",1000,sam_callback);

    ros::Subscriber motion_cmd_sub=n.subscribe<uxa_motion_control::motionCmdMsg>("motion_cmd_sub",200,sub_function_cmd);


    ROS_INFO("%s", "setup_motion_control");

    for(unsigned char i=0; i<25;i++)
    {
        samPos12.SAMPos12[i]=samPos12_hardware[i];
        samPos12.SAMMode[i]=i+1;
    }
    task.id=TASK_IDLE_;
    task.preId=TASK_IDLE_;
    while(ros::ok())
    {
        if(FlagTimer.Hz_50)
        {
            FlagTimer.Hz_50=0;
            //================
            //                unsigned char Trans_chr[1] ={ '0'};
            //                mySAM->Send_Serial_String(mySAM->Serial, Trans_chr, 1);
            //                  ROS_INFO("%s", "getFilteredData_50Hz");

        }
        if(FlagTimer.Hz_100)
        {
            FlagTimer.Hz_100=0;
            //====================
            task_handle();
        }
        if(FlagTimer.Hz_125)
        {
            FlagTimer.Hz_125=0;
            //===============

        }


        //==========================================


        ros::spinOnce();
        loop_rate.sleep();
        Timer_handler();
    }
    return 0;
}

void task_handle(){
    switch (task.id){
    case TASK_INIT_F_IDLE:
        if(task.startFlag==0){
            task.startFlag=1;
            ROS_INFO("task init: start");
            //===============

        }
        else if(sys_flag.feedback_sam_available)
        {
            sys_flag.feedback_sam_available=0;
            //====================================
            unsigned char count=0;
            for(unsigned char i=0;i<12;i++){
                if(currentSamAvail[i]==1)
                    count++;
            }

            if((count==12)&&(currentSamPos12[0]>0))
            {
                task.finishFlag=1;
                cout <<"init task done"<<endl;
                for (unsigned char i=0;i<12;i++)
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
        else if(task.finishFlag){
            ROS_INFO("task init: finish");
            task.finishFlag=0;
            task.startFlag=0;
            task.preId=TASK_INIT_F_IDLE;
            task.id=TASK_IDLE_;
            cout <<"finish init task "<<endl;
            //            mySam.setAllAverageTorque(sitdown_averageTorq,12);
            //            usleep(1000);
            //            mySam.setAllPDQuick(sitdown_samP,sitdown_samD,12);
            //            usleep(1000);
        }
        break;
    case TASK_SITDOWN_F_INIT:
        if(task.startFlag==0){
            task.startFlag=1;
            ROS_INFO("task sitdown: start");
            int beginpose[12];
            currentScene.mapHardToSoftPose(currentSamPos12,beginpose,12);
            currentScene.setUpMyScene(300,beginpose,pose_sitdown);
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
        }
        break;
    case TASK_STANDING_F_SITDOWN:
        if(task.startFlag==0){
            ROS_INFO("task standing: start");
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
            ROS_INFO("task standing: finish");
            task.finishFlag=0;
            task.startFlag=0;
            task.id=TASK_IDLE_;
            task.preId=TASK_STANDING_F_SITDOWN;
            sleep(1);
        }
        break;

    case TASK_STANDINGINIT_F_STANDING:
        if(task.startFlag==0){
            task.startFlag=1;
            currentScene.setUpMyScene(300,pose_standing_3,pose_init_walking);
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
        }
        break;

    case TASK_STANDING_F_STANDINGINIT:
        if(task.startFlag==0){
            task.startFlag=1;
            currentScene.setUpMyScene(300,pose_init_walking,pose_standing_3);
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
            ROS_INFO("task standing: finish");
            task.finishFlag=0;
            task.startFlag=0;
            task.id=TASK_IDLE_;
            task.preId=TASK_SITDOWN_F_STANDING;
            sleep(1);
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
                for(unsigned char i=0;i<12;i++)
                {
                    pos12_double[i]=*(currentScene.beginPose+i);
                    unsigned int b=*(currentScene.beginPose+i);
                    unsigned int a=*(currentScene.endPose+i);
                    delta_pos12[i]=((double)a-(double)b)/(double)currentScene.numOfFrame;
                }
            }
            else if(currentScene.frame<currentScene.numOfFrame)
            {
                for(unsigned char i=0;i<12;i++)
                {
                    pos12_double[i]+=delta_pos12[i];
                }
            }
            else{
                currentScene.flag.enable=0;
                currentScene.flag.finish=1;
                for(unsigned char i=0;i<12;i++)
                {
                    pos12_double[i]=*(currentScene.endPose+i);
                }
            }

            if(currentScene.flag.enable){
                for (unsigned char i=0;i<12;i++){
                    samPos12.SAMPos12[i]=(unsigned int)pos12_double[i];
                    samPos12.SAMMode[i]=i+1;
                    //                    ROS_INFO("ID: %d Mode: %d  SAMPos12: %d", i,samPos12.SAMMode[i],samPos12.SAMPos12[i]);
                }
                if(sys_flag.enable_sam)
                    sam_pos12_pub.publish(samPos12);
            }
        }
    }
}

void Scene_class::setBeginPose(const unsigned int *value)
{
    beginPose = (unsigned int*)value;
}

void Scene_class::setEndPose(const unsigned int *value)
{
    endPose = (unsigned int*)value;
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

void Scene_class::setUpMyScene(unsigned int fr, const int *beginpose, const int *endpose)
{
    unsigned int buf_beginPose[12];
    unsigned int buf_endPose[12];
    mapSoftToHardPose(beginpose,buf_beginPose,12);
    mapSoftToHardPose(endpose,buf_endPose,12);
    this->setNumOfFrame(fr);
    this->setBeginPose(buf_beginPose);
    this->setEndPose(buf_endPose);
    this->frame=0;
    this->flag.start=0;
    this->flag.finish=0;
    this->flag.enable=1;
    this->flag.delay=0;
}

void Scene_class::setFrame(unsigned int value)
{
    frame = value;
}

void Scene_class::mapSoftToHardPose(const int *softwarePose, unsigned int *hardwarePose, unsigned char size)
{
    for(unsigned char i=0;i<size;i++)
    {
        *(hardwarePose+i)=(unsigned int)(*(softwarePose+i)+(int)samPos12_hardware[i]);
    }
}

void Scene_class::mapHardToSoftPose(unsigned int *hardwarePose, int *softwarePose, unsigned char size)
{
    for(unsigned char i=0;i<size;i++)
    {
        *(softwarePose+i)=(unsigned int)((int)*(hardwarePose+i)-(int)samPos12_hardware[i]);
    }
}
