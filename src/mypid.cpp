
#include "uxa_motion_control/mypid.h"

myPID::myPID()
{
    reset_parameters();
}

void myPID::PID_controller(double feeback)
{

    p_er=( KP)+( KI)*( sampling_time)/(2.0)+( KD)/( sampling_time);
    p_pre_er=-( KP)+( KI)*( sampling_time)/(2.0)-(2.0)*( KD)/( sampling_time);
    p_pre_pre_er=( KD)/( sampling_time);

    er= set_point-feeback;
    output +=  er* p_er +  pre_er* p_pre_er +  pre_pre_er* p_pre_pre_er;

    pre_pre_er= pre_er;
    pre_er= er;
}

void myPID::PD_controller(double feeback)
{
    p_er=( KP)+( KD)/( sampling_time);
    p_pre_er=-( KP)-(2.0)*( KD)/( sampling_time);
    p_pre_pre_er=( KD)/( sampling_time);

    er= set_point-feeback;
    output +=  er* p_er +  pre_er* p_pre_er +  pre_pre_er* p_pre_pre_er;

    pre_pre_er= pre_er;
    pre_er= er;
}

void myPID::PID_type_1(double feedback)
{
    er= set_point-feedback;
    P_term= KP* er;
    I_term +=  KI* sampling_time*( er+ pre_er)/2.0;
    I_term=constrain( I_term,- I_limit, I_limit);

    D_term =  KD*( er- pre_er)/ sampling_time;
    pre_er= er;

    output= P_term+ I_term+ D_term;
}

void myPID::PD_type_1(double feedback)
{
    er= set_point-feedback;
    P_term= KP* er;

    D_term =  KD*( er- pre_er)/ sampling_time;
    pre_er= er;

    output= P_term+ D_term;
}

void myPID::PID_type_3(double feedback)
{
    er= set_point-feedback;
    fb=feedback;

    P_term= KP* er;
    I_term +=  KI* sampling_time*( er+ pre_er)/2.0;
    I_term=constrain( I_term,- I_limit, I_limit);

    D_term =  KD*(- fb +  pre_fb)/ sampling_time;
    pre_er= er;
    pre_fb= fb;

    output= P_term+ I_term+ D_term;
}

void myPID::PD_type_3(double feedback)
{
    er= set_point-feedback;
    fb=feedback;

    P_term= KP* er;

    D_term =  KD*(- fb + pre_fb)/ sampling_time;
    pre_er= er;

    pre_fb= fb;

    output= P_term+ D_term;
}

void myPID::PID_type_4(double feedback, double filter)
{
    er= set_point-feedback;
    fb=feedback;
    fb_filter= fb_filter*filter+ fb*(1-filter);

    P_term= KP* er;
    I_term +=  KI* sampling_time*( er+ pre_er)/2.0;
    I_term=constrain( I_term,- I_limit, I_limit);

    D_term =  KD*(- fb_filter +  pre_fb_filter)/ sampling_time;
    D_term=constrain( D_term,- D_limit, D_limit);
    pre_er= er;
    pre_fb_filter= fb_filter;
    pre_fb= fb;

    output= P_term+ I_term+ D_term;
}
void myPID::PD_type_4(double feedback, double filter)
{
    er= set_point-feedback;
    fb=feedback;
    fb_filter= fb_filter*filter+ fb*(1-filter);

    P_term= KP* er;

    D_term =  KD*(- fb_filter + pre_fb_filter)/ sampling_time;
    D_term=constrain( D_term,- D_limit, D_limit);
    pre_er= er;
    pre_fb_filter= fb_filter;

    output= P_term+ D_term;
}

void myPID::PID_type_5(double feedback, double filter)
{
    er= set_point-feedback;
    fb=feedback;
    fb_filter= fb_filter*filter+ fb*(1-filter);

    P_term= KP* er;
    I_term +=  KI* sampling_time*( er+ pre_er)/2.0;
    I_term=constrain( I_term,- I_limit, I_limit);

    D_term =  KD*(- fb_filter +  pre_fb_filter)/ sampling_time;
    D_term=constrain( D_term,- D_limit, D_limit);
    pre_er= er;
    pre_fb_filter= fb_filter;
    pre_fb= fb;

    output= P_term+ I_term+ D_term;
    output=constrain(output,-output_limit,output_limit);
}

void myPID::reset_parameters(void)
{
    output=0;
    pre_er=0;
    pre_pre_er=0;
    pre_fb=0;
    pre_pre_fb=0;
    I_term=0;
    pre_fb_filter=0;
    fb_filter=0;
}

