% 1. 데이터 불러오기
filename = '1st_floor_newton_Barometer_1.txt';
% 데이터 테이블 형식으로 읽기 (헤더 자동 인식)
data = readtable(filename);

% 2. 데이터 추출 및 전처리
counter = data.Counter;
% TimeFine 값을 초(Seconds) 단위로 상대적으로 변환 (시작 시간을 0초로)
% 데이터 간격(100)을 고려하여 10kHz 타이머로 가정 (필요시 센서 스펙에 맞게 수정)
time_sec = (data.TimeFine - data.TimeFine(1)) / 10000.0; 

acc_x = data.Acc_X;    
acc_y = data.Acc_Y;
acc_z = data.Acc_Z;
pressure = data.Pressure;
temp_c = data.Temp;



% 3. 기압 고도(Altitude) 계산 (SI 단위 적용)

% 유도했던 공식 적용: H = (T0 / 0.0065) * [ 1 - (P / P0)^0.190263 ]
EXPONENT = 0.190263;
P0 = 100684.63 / ((1 - (59.0/44330.77))^(1/EXPONENT));
altitude = 44330.77.* (1.0 - (pressure ./ P0).^EXPONENT);

% 4. 데이터 플롯팅 (Plotting)

% % 4-1. 가속도 데이터 플롯
% figure(1),
% plot(time_sec, acc_x, 'r', 'LineWidth', 1.2); hold on;
% plot(time_sec, acc_y, 'g', 'LineWidth', 1.2);
% plot(time_sec, acc_z, 'b', 'LineWidth', 1.2);
% title('Raw Accelerometer Data');
% ylabel('Acceleration (m/s^2)');
% legend('Acc X', 'Acc Y', 'Acc Z', 'Location', 'best');
% grid on;

% 4-2. 기압 및 온도 데이터 플롯 (이중 Y축 사용)
figure(1),
% yyaxis left;
plot(time_sec, pressure, 'k-', 'LineWidth', 1.2);
ylabel('Pressure (Pa)');
ylim([100620+45 100655+45]);
% yyaxis right;
% plot(time_sec, temp_c, 'm--', 'LineWidth', 1.2);
% ylabel('Temperature (\circC)');
title('Raw Pressure [1st Floor]');
grid on;

% % 4-3. 계산된 고도 데이터 플롯
% figure(2),
% plot(time_sec, smooth(altitude), 'b-', 'LineWidth', 1.5);
% title('Calculated Relative Altitude');
% xlabel('Time (seconds)');
% ylabel('Altitude (m)');
% grid on;

% 1. 데이터 평활화 및 통계값 계산
figure(2),
smoothed_alt = altitude;     % 스무딩 처리된 고도
mean_alt = mean(smoothed_alt);       % 고도 평균값 계산 (Raw 데이터 평균을 원하면 mean(altitude) 사용)
upper_bound = mean_alt + 0.5;        % +0.5m 상한선
lower_bound = mean_alt - 0.5;        % -0.5m 하한선

% 2. 고도 데이터 플롯
plot(time_sec, smoothed_alt, 'b-', 'LineWidth', 0.5, 'DisplayName', 'Altitude');
hold on; % 기존 그래프 위에 선을 겹쳐 그리기 위해 hold on 설정

% 3. 평균값 및 +-0.5m 범위 가로선 추가 (최신 매트랩 R2018b 이상 권장)
yline(mean_alt, 'r--', 'LineWidth', 3, 'DisplayName', sprintf('Mean: %.2f m', mean_alt));
yline(upper_bound, 'g--', 'LineWidth', 3, 'DisplayName', '+0.5m Bound');
yline(lower_bound, 'g--', 'LineWidth', 3, 'DisplayName', '-0.5m Bound');

% 4. 그래프 속성 설정
title('1st Floor Altitude (100Hz)');
xlabel('Time (seconds)');
ylabel('Altitude (m)');
legend('Location', 'best'); % 범례 표시
% ylim([56 68]);
grid on;
hold off; % 다음 그래프 그릴 때 겹치지 않도록 hold off
