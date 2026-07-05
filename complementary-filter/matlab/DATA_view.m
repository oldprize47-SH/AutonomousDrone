clear; clc; close all; % 기존 변수 및 창 초기화

% 1. 파일명 및 층별 라벨 설정 (반복문을 위한 Cell 배열)
filenames = {'1st_floor_newton_Barometer_1.txt', ...
             '2nd_floor_newton_Barometer_1.txt', ...
             '3rd_floor_newton_Barometer_2.txt'};
floor_names = {'1st Floor', '2nd Floor', '3rd Floor'};

floor_altitude = [59 63.08 66.53];


% 평균 고도 저장을 위한 배열 초기화
mean_alts = zeros(1, 3);
EXPONENT = 0.190263;
Temp_init = 26.043;

% 2. 반복문을 통한 개별 층 데이터 처리 및 플롯
for i = 1:length(filenames)
    % 데이터 불러오기
    data = readtable(filenames{i});
    
    % 데이터 추출 및 시간축 설정
    time_sec = (data.TimeFine - data.TimeFine(1)) / 10000.0; 
    pressure = data.Pressure;
    temp = data.Temp;

    % 기압 고도(Altitude) 계산
    T0 = (Temp_init + 273.15) + 0.0065*floor_altitude(1);
    P0 = 100680 / ((1 - (floor_altitude(1)/(T0/0.0065)))^(1/EXPONENT));
    altitude = (T0/0.0065).* (1.0 - (pressure ./ P0).^EXPONENT);
    
    % 기압 및 고도 평균값 계산
    mean_pressure = mean(pressure); 
    mean_alts(i) = mean(altitude);
    
    upper_bound_p = mean_pressure + 8;
    lower_bound_p = mean_pressure - 8;

    upper_bound = mean_alts(i) + 0.5;
    lower_bound = mean_alts(i) - 0.5;
    
    % --- 창 번호 계산 (층마다 2개씩 할당) ---
    fig_pressure = (i - 1) * 2 + 1; % 1, 3, 5
    fig_altitude = (i - 1) * 2 + 2; % 2, 4, 6
    
    % --- [독립된 창 1] Raw Pressure 플롯 ---
    figure(fig_pressure);
    plot(time_sec, pressure, 'k-', 'LineWidth', 0.5, 'DisplayName', 'Raw Pressure');
    hold on;

    yline(mean_pressure, 'r--', 'LineWidth', 2, 'DisplayName', sprintf('Mean: %.2f Pa', mean_pressure));
    yline(upper_bound_p, 'b--', 'LineWidth', 2, 'DisplayName', '+8 Pa Bound');
    yline(lower_bound_p, 'b--', 'LineWidth', 2, 'DisplayName', '-8 Pa Bound');
    
    title(sprintf('%s Raw Pressure (100Hz)', floor_names{i}));
    xlabel('Time (seconds)');
    ylabel('Pressure (Pa)');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % --- [독립된 창 2] Calculated Altitude 플롯 ---
    figure(fig_altitude);
    plot(time_sec, altitude, 'b-', 'LineWidth', 0.5, 'DisplayName', 'Altitude');
    hold on;
    
    yline(mean_alts(i), 'r--', 'LineWidth', 3, 'DisplayName', sprintf('Mean: %.2f m', mean_alts(i)));
    yline(upper_bound, 'g--', 'LineWidth', 2, 'DisplayName', '+0.5m Bound');
    yline(lower_bound, 'g--', 'LineWidth', 2, 'DisplayName', '-0.5m Bound');
    
    title(sprintf('%s Calculated Altitude', floor_names{i}));
    xlabel('Time (seconds)');
    ylabel('Altitude (m)');
    legend('Location', 'best');
    grid on;
    hold off;
end

% 3. 층별 고도 평균값 및 차이 요약 플롯 (Figure 7)
figure(7);
% 막대 그래프로 층별 평균 고도 시각화
b = bar(1:3, mean_alts, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k', 'LineWidth', 1.5);
hold on;

% 각 막대 위에 정확한 평균 고도 수치 표시
for i = 1:3
    text(i, mean_alts(i) + 0.2, sprintf('%.2f m', mean_alts(i)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontSize', 12, 'FontWeight', 'bold');
end

% 층간 고도 차이(Difference) 계산
diff_1to2 = mean_alts(2) - mean_alts(1);
diff_2to3 = mean_alts(3) - mean_alts(2);

% 차이값(Delta)을 그래프 막대 사이에 박스 형태로 표시
% 1층 -> 2층 차이
text(1.5, mean([mean_alts(1), mean_alts(2)]), sprintf('\\Delta = %.2f m', diff_1to2), ...
    'HorizontalAlignment', 'center', 'BackgroundColor', 'y', 'EdgeColor', 'k', 'FontSize', 11);
% 2층 -> 3층 차이
text(2.5, mean([mean_alts(2), mean_alts(3)]), sprintf('\\Delta = %.2f m', diff_2to3), ...
    'HorizontalAlignment', 'center', 'BackgroundColor', 'y', 'EdgeColor', 'k', 'FontSize', 11);

% 요약 그래프 속성 설정
set(gca, 'XTick', 1:3, 'XTickLabel', floor_names);
title('Average Altitude per Floor and Differences');
ylabel('Mean Altitude (m)');
% Y축 범위를 텍스트가 잘리지 않도록 동적으로 약간 여유 있게 설정
ylim([min(mean_alts) - 2, max(mean_alts) + 3]);
grid on;
hold off;