#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "globVariable.h"
#include "kinematics.h"
#include "trajectory.h"
#include <mujoco/mujoco.h>
#include "filter.h"
#include <vector>

using namespace std;

//전역변수로 Leg_num 있어야함, PID제어기 몇개 인지도 있어야함

class controller
{

private:
    
    // Pos_PID
    Vector2d Kp_pos; // [Leg_num][dimension] ->ex) [RL][r,theta]
    Vector2d Kd_pos;
    double cut_off_D_pos;
    Vector2d PID_output_pos;
    Vector2d error_pos;
    Vector2d error_old_pos;
    Vector2d error_dot_pos;
    Vector2d error_dot_old_pos;

    // Vel_PID
    Vector2d Kp_vel; // [Leg_num][dimension] ->ex) [RL][r,theta]
    Vector2d Kd_vel;
    double cut_off_D_vel;
    Vector2d PID_output_vel;
    Vector2d error_vel;
    Vector2d error_old_vel;
    Vector2d error_dot_vel;
    Vector2d error_dot_old_vel;

    //Admittance
    double ad_M;
    double ad_B;
    double ad_K;
    Vector2d deltaPos;
    Vector2d deltaPos_old;
    Vector2d deltaPos_old2;

    //DOB
    Vector2d rhs_dob;
    Vector2d rhs_dob_old;
    Vector2d lhs_dob;
    Vector2d lhs_dob_old;
    double lhs_dob_LPF[NDOF_LEG];
    double lhs_dob_LPF_old[NDOF_LEG];
    double rhs_dob_LPF[NDOF_LEG];
    double rhs_dob_LPF_old[NDOF_LEG];
    Vector2d tauDist_hat;
    double cut_off_dob;

    //FOB
    Vector2d rhs_fob;
    Vector2d rhs_fob_old;
    Vector2d lhs_fob_LPF;
    Vector2d lhs_fob_LPF_old;
    Vector2d rhs_fob_LPF;
    Vector2d rhs_fob_LPF_old;
    Vector2d tauExt_hat;
    Vector2d forceExt_hat;
    Vector2d forceExt_hat_old;
    Vector2d forceExt_hat_old2; 
    double cut_off_fob;



public:
    controller(); //생성자
    ~controller(); //소멸자
    
    void pid_gain_pos(double kp, double kd, double cut_off_pos, int flag);
    void pid_gain_vel(double kp, double kd, double cut_off_vel, int flag);
    Vector2d PID_pos(StateModel_* state_model);
    void PID_vel(StateModel_* state_model);
    void admittanceCtrl(StateModel_* state_model, double m, double b, double k, int flag);
    Vector2d DOBRW(StateModel_* state_model, double cut_off ,int flag);
    void FOBRW(StateModel_* state_model, double cut_off);
    void ctrl_update(); // 이걸 사용할지에 대해서 생각해보기
    
    // Data Logging
    vector<Eigen::VectorXd> Data_Return(controller& ctrl); 
    // Getter function(start variable 0)
    Vector2d get_PID_output_pos(){return PID_output_pos;};
    Vector2d get_PID_output_vel(){return PID_output_vel;};
    Vector2d get_error_pos(){return error_pos;};
    Vector2d get_deltaPos(){return deltaPos;};
    
    

};
#endif // CONTROLLER_H_

