%% Running Mujoco Simulation

clear all
clc
close all
system('./run_linux')
%% Getting Mujoco Simulation Data

FL_data = './data/data_FL.csv';
FR_data = './data/data_FR.csv';
RL_data = './data/data_RL.csv';
RR_data = './data/data_RR.csv';

FL_T = readtable(FL_data);
FR_T = readtable(FR_data);
RL_T = readtable(RL_data);
RR_T = readtable(RR_data);

FL_VariableNames = FL_T.Properties.VariableNames;
FR_VariableNames = FR_T.Properties.VariableNames;
RL_VariableNames = RL_T.Properties.VariableNames;
RR_VariableNames = RR_T.Properties.VariableNames;

FL_Arr = table2array(FL_T);
FR_Arr = table2array(FR_T);
RL_Arr = table2array(RL_T);
RR_Arr = table2array(RR_T);

[FL_m,FL_n] = size(FL_Arr);
[FR_m,FR_n] = size(FR_Arr);
[RL_m,RL_n] = size(RL_Arr);
[RR_m,RR_n] = size(RR_Arr);
%% Plot for FL Leg (Front Left Leg)

figure(1);
subplot(2,2,1);
plot(FL_Arr(:,1), FL_Arr(:,2), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(FL_Arr(:,1), FL_Arr(:,3), 'Color', 'm', 'LineWidth', 3);
xlim([0 1.2]);
xlabel('time (sec)');
ylabel('FL GRF (N)');
legend('real', 'FOB')

subplot(2,2,2);
plot(FL_Arr(:,1), FL_Arr(:,3)-FL_Arr(:,2), 'Color', 'c', 'LineWidth', 3);
xlim([0 1.2]);
xlabel('time (sec)');
ylabel('FL GRF (N)');
legend('FOB error')

subplot(2,2,3);
plot(FL_Arr(:,1), FL_Arr(:,4), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(FL_Arr(:,1), FL_Arr(:,5), 'Color', 'r', 'LineWidth', 3);
xlim([0 1.2]);
xlabel('time (sec)');
ylabel('FL r (m)');
legend('reference', 'real')

subplot(2,2,4)
plot(FL_Arr(:,1), FL_Arr(:,6), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(FL_Arr(:,1), FL_Arr(:,7), 'Color', 'r', 'LineWidth', 3);
xlim([0 1.2]);
xlabel('time (sec)');
ylabel('FL theta (rad)');
legend('reference', 'real')