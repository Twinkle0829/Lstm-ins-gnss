
clc
close all
clear
addpath(genpath('E:\2020\IAC\LSTM EKF\EKF LSTM\conversions'));
addpath(genpath('E:\2020\IAC\LSTM EKF\EKF LSTM\updata'));
addpath(genpath('E:\2020\IAC\LSTM EKF\EKF LSTM\data'));
addpath(genpath('E:\2020\IAC\LSTM EKF\EKF LSTM\KF'));
addpath(genpath('E:\2020\IAC\LSTM EKF\EKF LSTM\jichu'));




%% CONVERSION CONSTANTS
G =  9.80665;       % Gravity constant, m/s^2
G2MSS = G;          % g to m/s^2
MSS2G = (1/G);      % m/s^2 to g

D2R = (pi/180);     % degrees to radians
R2D = (180/pi);     % radians to degrees

KT2MS = 0.514444;   % knot to m/s
MS2KMH = 3.6;       % m/s to km/h

%% LOAD  DATA

%fprintf(' loading reference data... \n')
load ref
%% EKINOX IMU 
fprintf(' loading MPU-6000 IMU data... \n')
load mpu6000_imu
%% EKINOX GNSS 
fprintf(' loading Ekinox GNSS data... \n')
load ekinox_gnss
%load lstm
%% Print navigation time
to = (ref.t(end) - ref.t(1));
fprintf('navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%loa
%load lstm_data 
%% INS/GNSS integration

%if strcmp(INS_GNSS, 'ON')
    
    fprintf('NaveGo: INS/GNSS integration... \n')
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
   [nav_mpu6000] = ins_gnss(mpu6000_imu, ekinox_gnss, 'quaternion'); %
  % [nav_mpu6000] = ins_gnss(mpu6000_imu, lstm, 'quaternion');
    % ---------------------------------------------------------------------
      %  tstlstm=lstm_datachange(lstm_data);
%else
    
  %  load nav_mpu6000
 %end
   save nav_mpu6000.mat nav_mpu6000

  %  save naesr.mat naesr
%% ANALYZE PERFORMANCE FOR A CERTAIN PART OF THE INS/GNSS DATASET  分析某一部分INS/GNSS数据集的性能

tmin_rmse = ref.t(1); 
tmax_rmse = ref.t(end); 

% Sincronize REF data to tmin and tmax
idx  = find(ref.t > tmin_rmse, 1, 'first' );
fdx  = find(ref.t < tmax_rmse, 1, 'last' );
if(isempty(idx) || isempty(fdx))
    error('ref: empty index')
end

ref.t       = ref.t    (idx:fdx);
ref.roll    = ref.roll (idx:fdx);
ref.pitch   = ref.pitch(idx:fdx);
ref.yaw     = ref.yaw  (idx:fdx);
ref.lat     = ref.lat  (idx:fdx);
ref.lon     = ref.lon  (idx:fdx);
ref.h       = ref.h    (idx:fdx);
ref.vel     = ref.vel  (idx:fdx, :);

%% Interpolate INS/GNSS dataset 

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset. 根据参考数据集对INS/GNSS估计和GNSS数据进行插值。

 [nav_r,  ref_n] = navego_interpolation (nav_mpu6000, ref);
 [gnss_r, ref_g] = navego_interpolation (ekinox_gnss, ref);

%% Print navigation time

%to = (ref.t(end) - ref.t(1));

%fprintf('navigation time under analysis is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% Print RMSE from INS/GNSS data

% rmse_v = print_rmse (nav_r, gnss_r, ref_n, ref_g, 'Ekinox INS/GNSS');

%% Save RMSE to CVS file

% csvwrite('ekinox.csv', rmse_v);

%% PLOT

%if (strcmp(PLOT,'ON'))
    
  navego_plot (ref, ekinox_gnss, nav_mpu6000, gnss_r, nav_r, ref_g, ref_n)
   % navego_plot (ref, lstm, nav_mpu6000, gnss_r, nav_r, ref_g, ref_n)

%end
