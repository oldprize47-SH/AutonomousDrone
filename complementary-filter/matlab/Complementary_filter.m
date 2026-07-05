clear; clc; close all;

% For Select Files
filenames = {'260321_elebator_test.txt'};

% Read
data  = readtable(filenames{1});

pressure    = data.Pressure ;
AccZ        = data.Acc_Z    ;
temp        = data.Temp     ;
time        = (data.TimeFine - data.TimeFine(1)) / 10000.0; 

% Initial Parameter
TRUE_ALTITUDE   = 59.0            ; % [m]
GAMMA           = 0.0065          ; %[c^o/m]
EXPONENT        = 0.190263        ;
temp_init       = temp(1) + 273.15; % [K]
pressure_init   = pressure(1)     ;

g0        =  -9.7803267714; % [m/sec^2]
R         =  6378137     ; % [m]
T0        =  temp_init + (GAMMA * TRUE_ALTITUDE);
P0        =  pressure_init / ((1 - ( TRUE_ALTITUDE/(T0/GAMMA)) )^(1/EXPONENT));

% Gravity model
g_0       = 9.7803253359 * (1 + 0.00193185265241*sin(L_rad)^2) / sqrt(1 - 0.00669437999014*sin(L_rad)^2);
gl        = 


% Cal Altitude
altitude  =  (T0/GAMMA).* (1.0 - (pressure ./ P0).^EXPONENT);
H_initial =  altitude(1);


% Setting data
a_D     = [time, -(AccZ)];
H_aid   = [time, altitude]    ;


% For Complementary Filter
tau     = 3           ;
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

plot(time, altitude, 'b-', 'LineWidth', 1.5); 
hold on;

plot(t_sim, H_estimated, 'r-', 'LineWidth', 2.0); 

xlabel('Time (sec)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Altitude (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('Complementary Filter: Raw Barometer vs Estimated Altitude', 'FontSize', 14);
legend('Raw Barometer (H_{aid})', 'Estimated Altitude (H)', 'Location', 'best');
grid on;

xlim([0 sim_time]);