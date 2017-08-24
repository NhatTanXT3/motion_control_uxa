#ifndef MY_PID_H
#define MY_PID_H

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class myPID
{
public:
    myPID();
    double set_point;
    double KP;
    double KD;
    double KI;
    double output;
    double er;
    double pre_er;
    double pre_pre_er;

    double p_er;
    double p_pre_er;
    double p_pre_pre_er;


    double sampling_time;

    double P_term;
    double I_term;
    double D_term;
    double I_limit;
    double D_limit;
    double output_limit;

    double fb;//feedback
    double pre_fb;
    double pre_pre_fb;
    double fb_filter;
    double pre_fb_filter;



    void PID_controller(double feeback);
    void PD_controller(double feeback);

    void PID_type_1(double feedback);
    void PD_type_1(double feedback);


    void PID_type_3(double feedback);
    void PD_type_3(double feedback);


    void PID_type_4(double feedback,double filter);
    void PD_type_4(double feedback,double filter);

    void PID_type_5(double feedback,double filter);

//    void PD_type_4(double feedback,double filter);

    void reset_parameters(void);

};

#endif // PID_H
