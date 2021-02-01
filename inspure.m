
%
load ekinox_imu
load ekinox_gnss
imu=ekinox_imu;
gnss=ekinox_gnss;
att_mode='quaternion';
%lstm_data.wb=imu.wb(:,:);
fprintf('wb integration... \n');
%lstm_data.fb=imu.fb(:,:);
fprintf('fb integration... \n');

%% ZUPT detection algorithm

zupt = false;

%% PREALLOCATION ?预先配置

% Constant matrices 常数矩阵
I = eye(3);
O = zeros(3);

% Length of INS time vector INS时间向量的长度
% LI = length(imu.t);
LI = length(imu.t);
LG = length(gnss.t);
%lstm_data.t=imu.t;
% Length of GNSS time vector GNSS时间矢量的长度
%LG = length(gnss.t);

% Attitude
roll_e  = zeros (LI, 1);
pitch_e = zeros (LI, 1);
yaw_e   = zeros (LI, 1);
%     yawm_e  = zeros (Mi, 1);

% Initialize estimates at INS time = 1
roll_e(1)  = imu.ini_align(1);
pitch_e(1) = imu.ini_align(2);
yaw_e(1)   = imu.ini_align(3);
%     yawm_e(1)  = imu.ini_align(3);
DCMnb = euler2dcm([roll_e(1); pitch_e(1); yaw_e(1);]);
DCMbn = DCMnb';
qua   = euler2qua([roll_e(1) pitch_e(1) yaw_e(1)]);

% Velocities
vel_e   = zeros (LI, 3);

% Initialize estimates at INS time = 1 初始化估算在INS时间= 1
vel_e(1,:) = gnss.vel(1,:);

% Positions
lat_e    = zeros (LI,1);
lon_e    = zeros (LI,1);
h_e      = zeros (LI, 1);

% Initialize estimates at INS time = 1
h_e(1)   = gnss.h(1);
lat_e(1) = gnss.lat(1);
lon_e(1) = gnss.lon(1);

% Biases
gb_dyn = imu.gb_dyn';
ab_dyn = imu.ab_dyn';

% Initialize Kalman filter matrices

% Prior estimates
kf.xi = [ zeros(1,9), imu.gb_dyn, imu.ab_dyn ]';  % Error vector state
kf.Pi = diag([imu.ini_align_err, gnss.stdv, gnss.std, imu.gb_dyn, imu.ab_dyn].^2);

kf.R  = diag([gnss.stdv, gnss.stdm].^2);
kf.Q  = diag([imu.arw, imu.vrw, imu.gb_psd, imu.ab_psd].^2);

fb_corrected = (imu.fb(1,:)' + ab_dyn );
fn = (DCMbn * fb_corrected);

% Vector to update matrix F
upd = [gnss.vel(1,:) gnss.lat(1) gnss.h(1) fn'];

% Update matrices F and G
[kf.F, kf.G] = F_update(upd, DCMbn, imu);

[RM,RN] = radius(gnss.lat(1));
Tpr = diag([(RM + gnss.h(1)), (RN + gnss.h(1)) * cos(gnss.lat(1)), -1]);  % radians-to-meters
        
% Update matrix H
kf.H = [ O I O O O ;
    O O Tpr O O ; ];
kf.R = diag([gnss.stdv gnss.stdm]).^2;
kf.z = [ gnss.stdv, gnss.stdm ]';
        
% Propagate prior estimates to get xp(1) and Pp(1)
kf = kf_update( kf );

% PENDING: UD filter matrices
% [Up, Dp] = myUD(S.P);
% dp = diag(Dp);

% DEC = 0.5 * 180/pi;             % Magnetic declination (radians)

% Kalman filter matrices for later performance analysis
xi = zeros(LG, 15);        % Evolution of Kalman filter a priori states, xi
xp = zeros(LG, 15);        % Evolution of Kalman filter a posteriori states, xp
z = zeros(LG, 6);          % INS/GNSS measurements
v = zeros(LG, 6);          % Kalman filter innovations
b  = zeros(LG, 6);         % Biases compensantions after Kalman filter correction

A  = zeros(LG, 225);       % Transition-state matrices, A
Pi = zeros(LG, 225);       % Priori covariance matrices, Pi
Pp = zeros(LG, 225);       % Posteriori covariance matrices, Pp
K  = zeros(LG, 90);       % Kalman gain matrices, K
S  = zeros(LG, 36);       % Innovation matrices, S

% Initialize matrices for Kalman filter performance analysis
xp(1,:) = kf.xp';
Pp(1,:) = reshape(kf.Pp, 1, 225);
b(1,:)  = [imu.gb_sta, imu.ab_sta];

% INS (IMU) time is the master clock
for i = 2:LI
    
    %% INERTIAL NAVIGATION SYSTEM (INS)
    
    % Print a dot on console every 10,000 INS executions
    if (mod(i,10000) == 0), fprintf('. ');  end
    % Print a return on console every 200,000 INS executions
    if (mod(i,200000) == 0), fprintf('\n'); end
    
    % IMU sampling interval
    dti = imu.t(i) - imu.t(i-1);
    
    % Inertial sensors corrected with KF biases estimation
    wb_corrected = (imu.wb(i,:)' + gb_dyn );
    fb_corrected = (imu.fb(i,:)' + ab_dyn );
    
    % Turn-rates update
    omega_ie_n = earthrate(lat_e(i-1));
    omega_en_n = transportrate(lat_e(i-1), vel_e(i-1,1), vel_e(i-1,2), h_e(i-1));
    
    % Attitude update
    [qua_n, DCMbn, euler] = att_update(wb_corrected, DCMbn, qua, ...
        omega_ie_n, omega_en_n, dti, att_mode);
    roll_e(i) = euler(1);
    pitch_e(i)= euler(2);
    yaw_e(i)  = euler(3);
    qua = qua_n;
    
  %  lstm_data.roll_e = roll_e(1:i, :);
   % lstm_data.yaw_e = yaw_e(1:i,:);
  %  lstm_data.pitch_e= pitch_e(1:i,:);
    
    % Gravity update
    gn = gravity(lat_e(i-1), h_e(i-1));
    
    % Velocity update
    fn = (DCMbn * fb_corrected);
    vel_n = vel_update(fn, vel_e(i-1,:), omega_ie_n, omega_en_n, gn', dti);
    vel_e (i,:) = vel_n;
   % lstm_data.vel=vel_e(1:i,:);
%    printf('vel integration... \n');
    
    % Position update
    pos = pos_update([lat_e(i-1) lon_e(i-1) h_e(i-1)], vel_e(i,:), dti);
    lat_e(i) = pos(1);
    lon_e(i) = pos(2);
    h_e(i)   = pos(3);
    
     % lstm_data.t=imu.t(1:i, :);
 % lstm_data.lat(i)=lat_e(i);
 % lstm_data.lon(i)= lon_e(i) ; 
  %lstm_data.h(i)= h_e(i);
    % PENDING. Magnetic heading update
    %         yawm_e(i) = hd_update (imu.mb(i,:), roll_e(i),  pitch_e(i), D);

    %% KALMAN FILTER UPDATE
   

end
     %%
 function aa()% ZUPT detection algorithm
    idz = floor( gnss.zupt_win / dti ); % Index to set ZUPT window time
    
    if ( i > idz )
        
        vel_m = mean (vel_e(i-idz:i , :));
        
        if (abs(vel_m) <= gnss.zupt_th)
            
            % Alternative attitude ZUPT correction
            % roll_e(i) = (roll_e(i-idz , :));
            % pitch_e(i)= (pitch_e(i-idz , :));
            %                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
            % yaw_e(i)  = (yaw_e(i-idz, :));
            
            roll_e(i) = mean (roll_e(i-idz:i , :));
            pitch_e(i)= mean (pitch_e(i-idz:i , :));
            yaw_e(i)  = mean (yaw_e(i-idz:i , :));
            
            lat_e(i) = mean (lat_e(i-idz:i , :));
            lon_e(i) = mean (lon_e(i-idz:i , :));
            h_e(i)   = mean (h_e(i-idz:i , :));
            
            zupt = true;

        end
    end
 end