clear; clc; close all;


% For Select Files
filenames = {'260327_filter_test_linear.txt', '260327_filter_test_nonlinear.txt'};

% Read
data1   = readtable(filenames{1});
data2   = readtable(filenames{2});

time1   = (data1.Elapsed - data1.Elapsed(1));
time2   = (data2.Elapsed - data2.Elapsed(1));

figure,
plot(time1, data1.Filtered_Alt);

figure,
plot(time2, data2.Filtered_Alt);