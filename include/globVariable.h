#ifndef __GLOBVARIABLE_H__
#define __GLOBVARIABLE_H__

#include <stdbool.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#define NDOF_TRUNK 3 // #(DoF) of trunk
#define NDOF_LEG 2   // #(DoF) of leg
#define NUM_LEG 4

const double Ts = 0.0001; // sampling period
const double g = 9.81;    // gravitational accel.
const double pi = 3.141592;

/* Trunk Dimension */
extern const double width_trunk;
extern const double length_trunk;
extern const double length_front;
extern const double length_rear;
extern const double height_trunk;

extern double jacb_trunk_pos[NUM_LEG * NUM_LEG];
extern double jacb_trunk_pos_inv[NUM_LEG * NUM_LEG];
extern double jacb_trunk_vel[NUM_LEG * NDOF_TRUNK];

/* Trunk States */
extern double pos_trunk_act[6];
extern double vel_trunk_act[6];
extern double rot_mat_trunk[9];
extern double rot_mat_trunk_act[9];

extern double pos_trunk_des[NDOF_TRUNK];
extern double pos_trunk_des_old[NDOF_TRUNK];
extern double pos_trunk_est[NDOF_TRUNK];
extern double pos_trunk_est_old[NDOF_TRUNK];

extern double vel_trunk_des[NDOF_TRUNK];
extern double vel_trunk_des_old[NDOF_TRUNK];
extern double vel_trunk_est[NDOF_TRUNK];
extern double vel_trunk_est_old[NDOF_TRUNK];

extern double error_pos_trunk[NDOF_TRUNK];
extern double error_pos_trunk_old[NDOF_TRUNK];
extern double error_dot_pos_trunk[NDOF_TRUNK];
extern double error_dot_pos_trunk_old[NDOF_TRUNK];

extern double error_vel_trunk[NDOF_TRUNK];
extern double error_vel_trunk_old[NDOF_TRUNK];

extern double ctrl_input_RW_from_trunk[NUM_LEG];

struct StateModel_
{   
    //state_model에 새로 생성되는 얘들 초기화 및 update 확인하기
    // old,current need to match with vector or matrix(matrix)

    /* Trunk States */
    /* Joint Coordinates */
    double q[NDOF_LEG];    // Serial Coordinates
    double q_old[NDOF_LEG]; // gravity,coriolis ��� ������ ����

    double q_bi[NDOF_LEG]; // biarticular joint angle
    double q_bi_old[NDOF_LEG];

    Vector2d qdot_bi;  // biarticular joint angular vel (sensor)
    Vector2d qdot_bi_old;
    Vector2d qddot_bi; // biarticular joint angular acc (sensor)

    Vector2d qdot_bi_tustin; // biarticular joint angular vel (derivative)
    Vector2d qdot_bi_tustin_old;

    Vector2d qddot_bi_tustin; // biarticular joint angular acc (derivative)
    Vector2d qddot_bi_tustin_old;

    Vector2d tau_bi; // (Biarticular) joint torques
    Vector2d tau_bi_old;
    Vector2d tau_bi_old2;

    /* Rotating Workspace Coordinates */
    double r0;              // initial leg length
    Vector2d posRW; // RW position
    Vector2d posRW_old;
    Vector2d posRW_ref; // RW position reference
    Vector2d posRW_ref_old;
    Vector2d posRW_ref_old2;
    // double error_pos[NDOF_LEG];
    // double error_pos_old[NDOF_LEG];

    Vector2d velRW; // RW velocity
    Vector2d velRW_old;
    Vector2d velRW_ref; // RW velocity reference
    Vector2d velRW_ref_old;
    // double error_vel[NDOF_LEG];
    // double error_vel_old[NDOF_LEG];

    double ctrl_input_RW[NDOF_LEG]; // control input
    double ctrl_input_RW_old[NDOF_LEG];

    /* Jacobian (Rotating Workspace) */
    Matrix2d jacbRW;
    Matrix2d jacbRW_trans;
    Matrix2d jacbRW_trans_inv;
    
    double touch_sensor;
    double tau_ff[NDOF_LEG];

    double time;

    //Make New version
    double cut_off_cal;
    Matrix2d Lamda_nominal_DOB;
    Matrix2d Lamda_nominal_FOB;
    Vector2d H; // Coriolis & Gravity term
    Vector2d H_old;

    // Added Values for Observer Comparing
    Vector2d GRF_FOB;
};


#endif // !__GLOBVARIABLE_H__