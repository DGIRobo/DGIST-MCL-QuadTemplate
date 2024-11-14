#ifndef DATALOGGING_H_
#define DATALOGGING_H_

#include "globVariable.h"

/***************** Data Logging *****************/
FILE* fid_FL;
FILE* fid_FR;
FILE* fid_RL;
FILE* fid_RR;
FILE* fid_Trunk;

int loop_index = 0;
const int data_frequency = 60; // frequency at which data is written to a file

char datapath_FL[] = "../data/data_FL.csv";
char datapath_FR[] = "../data/data_FR.csv";
char datapath_RL[] = "../data/data_RL.csv";
char datapath_RR[] = "../data/data_RR.csv";
char datapath_Trunk[] = "../data/data_trunk.csv";

// XML File Logging
char filename[] = "../data/scene.xml";

void init_save_data_leg(FILE* fid)
{
    // This function is called once and is used to get the headers
    // Write name of the variable here (header)
    // comma(,) should be omitted in the last line.

    fprintf(fid, "t, ");
    fprintf(fid, "real_GRF, FOB_GRF, ");

    fprintf(fid, "posRWr_ref, posRWr, posRWtheta_ref, posRWtheta, ");

    fprintf(fid, "tau_bi_m, tau_bi_b, qddot_m, qddot_b");

    // Don't remove the newline
    fprintf(fid, "\n");
}

void save_data_leg(const mjModel* m, mjData* d, StateModel_* state_model, vector<Eigen::VectorXd> Ctrl_data, FILE* fid, int leg_sensor_no)
{
    // This function is called at a set frequency,put data here.
    // Data here should correspond to headers in init_save_data()
    // Seperate data by a space %f followed by space
    // comma(,) should be omitted in the last line.
    double touch = d->sensordata[leg_sensor_no + 0];
    double grf_x = d->sensordata[leg_sensor_no + 1];
    double grf_y = d->sensordata[leg_sensor_no + 2];
    double grf_z = d->sensordata[leg_sensor_no + 3];

    double cartesian_grf_x = grf_x * cos(pi - state_model->q_bi[1]) - grf_y * sin(pi - state_model->q_bi[1]);
    double cartesian_grf_y = grf_x * sin(pi - state_model->q_bi[1]) + grf_y * cos(pi - state_model->q_bi[1]); 

    double grf_r = cartesian_grf_y * cos(state_model->posRW[1] - pi / 2) + cartesian_grf_x * sin(state_model->posRW[1] - pi / 2);
    double grf_thetar = cartesian_grf_y * sin(state_model->posRW[1] - pi / 2) - cartesian_grf_x * cos(state_model->posRW[1] - pi / 2);

    fprintf(fid, "%f, ", d->time);

    fprintf(fid, "%f, %f, ", grf_r, state_model->GRF_FOB[0]);

    fprintf(fid, "%f, %f, %f, %f, ", state_model->posRW_ref[0], state_model->posRW[0], state_model->posRW_ref[1], state_model->posRW[1]);

    fprintf(fid, "%f, %f, %f, %f", state_model->tau_bi[0], state_model->tau_bi[1], state_model->qddot_bi_tustin[0], state_model->qddot_bi_tustin[1]);

    // Don't remove the newline
    fprintf(fid, "\n");
}

void init_save_data_trunk(FILE* fid)
{
    // This function is called once and is used to get the headers
    // Write name of the variable here (header)
    // comma(,) should be omitted in the last line.

    fprintf(fid, "t,");
    fprintf(fid, "touch_FL, touch_FR, touch_RL, touch_RR");

    // Don't remove the newline
    fprintf(fid, "\n");
}

void save_data_trunk(const mjModel* m, mjData* d, FILE* fid)
{
    // This function is called once and is used to get the headers
    // Write name of the variable here (header)
    // comma(,) should be omitted in the last line.

    double touch_FL = d->sensordata[18];
    double touch_FR = d->sensordata[22];
    double touch_RL = d->sensordata[26];
    double touch_RR = d->sensordata[30];

    // touch sensor 넣기, GRF 측정값들 여기에 넣기
    fprintf(fid, "%f, ", d->time);
    fprintf(fid, "%f, %f, %f, %f ", touch_FL, touch_FR, touch_RL, touch_RR);

    // Don't remove the newline
    fprintf(fid, "\n");
};

#endif // DATALOGGING_H_