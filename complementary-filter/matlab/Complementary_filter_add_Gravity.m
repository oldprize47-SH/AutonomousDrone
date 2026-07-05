clear; clc; close all;

% For Select Files
filenames = {'260324_elebator_1.txt', '260324_elebator_2.txt'};

% Read
data   = readtable(filenames{2});
data_c = readtable("filter_result_1.txt");

pressure    = data.Pressure ;
AccZ        = data.Acc_Z    ;
temp        = data.Temp     ;
% time        = (data.TimeFine - data.TimeFine(1)) / 10000.0; 
time        = (data.SampleTimeFine - data.SampleTimeFine(1)) / 10000.0;

% Initial Parameter
TRUE_ALTITUDE   = 59.0            ; % [m]
GAMMA           = 0.0065          ; %[c^o/m] same as [K/m]
EXPONENT        = 0.190263        ;
temp_init       = mean(rmmissing(temp(1:800))) + 273.15    ; % [K]
pressure_init   = mean(pressure(1:800))         ;

g0        =  -9.7803267714; % [m/sec^2]
R         =  6378137      ; % [m]
T0        =  288.15       ; %temp_init + (GAMMA * TRUE_ALTITUDE);
P0        =  pressure_init / ((1 - ( TRUE_ALTITUDE/(T0/GAMMA)) )^(1/EXPONENT));

% Gravity model
L_deg       = 36.10363889   ; 
L_rad       = deg2rad(L_deg);
h_ellipse   = 88.330        ;
g0_model    = 9.7803267714 * (1 + 0.0052790414 * sin(L_rad)^2 ...
                                + 0.0000232718 * sin(L_rad)^4);
gl          = -g0_model/((1 + (h_ellipse/R))^2);

% Cal Altitude
altitude  =  (T0/GAMMA).* (1.0 - (pressure ./ P0).^EXPONENT);
H_initial =  altitude(1);


% Setting data
a_D     = [time,   (AccZ)]    ;
H_aid   = [time, altitude]    ;
H_filt  = [time,   data_c.Filtered_Alt]    ;

% For Complementary Filter
tau     = 5           ;
Omega_S = sqrt((g0)/R);

% Feedback Gain
C1 =                      3/tau;
C2 =  3/(tau^2) + 2*(Omega_S^2);
C3 =                  1/(tau^3);

%% Sim
sim_time = time(end); 

out = sim('Complementary_filter_Sim', 'StopTime', num2str(sim_time));

t_sim = out.tout;          % 시뮬링크가 계산한 시간 배열
H_estimated = out.H_LinearFilter;   % To Workspace 블록으로 받아온 추정 고도

figure('Name', 'Complementary Filter Result', 'Color', 'w');

% plot(t_sim, out.H_NoFilter, 'b--', 'LineWidth', 1.5); 
% hold on;

plot(t_sim, H_estimated, 'r--', 'LineWidth', 2.5); 
hold on;

plot(t_sim, out.H_NonLinearFilter , 'g-', 'LineWidth', 2.0); 
hold on;

plot(t_sim, out.H_c , 'b-', 'LineWidth', 2.0); 

xlabel('Time (sec)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Altitude (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('Complementary Filter: Altitude', 'FontSize', 14);
legend('Raw Barometer (H_{aid})', 'Linear Altitude (H)', 'Gravity model Altitude (H)', 'Location', 'best');
grid on;

xlim([0 sim_time]);