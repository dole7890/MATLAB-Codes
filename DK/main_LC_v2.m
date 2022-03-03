function varargout = main_LC(varargin)
% clc; clear all; close all;
%SCRIPT Loosely coupled INS/GNSS demo:
%   Profile_1 (60s artificial car motion with two 90 deg turns)
%   Tactical-grade IMU

[settings, IMU_errors, GNSS_config, LC_KF_config] = initSettings_LC();

% CONFIGURATION
% Input truth motion profile filename
input_profile_name = 'Profile_1.csv';
% Output motion profile and error filenames
output_profile_name = 'INS_GNSS_Demo_3_Profile.csv';
output_errors_name = 'INS_GNSS_Demo_3_Errors.csv';

% Attitude initialization error (deg, converted to rad; @N,E,D)
initialization_errors.delta_eul_nb_n = [-0.05;0.04;1]*settings.deg_to_rad; % rad

% Input truth motion profile from .csv format file
[in_profile,no_epochs,ok] = Read_profile(input_profile_name);
if ~ok
    return;
end %if

%% Read Data
% Truth Data
novatel = csvread('part6_edit.csv',2,0);

WN = novatel(1,1);
TOW = novatel(:,2);
% imu_truth = truth(:,[3:8,21:23]);
in_profile = [novatel(:,2),deg2rad(novatel(:,3:4)),novatel(:,5),novatel(:,19),novatel(:,18),-novatel(:,20),deg2rad(novatel(:,33)),deg2rad(novatel(:,32)),deg2rad(novatel(:,31)), novatel(:,21:23)];
%37: SD EAST 38: SD North 39: SD Height 40: SD Hor
% 41: SD VE 42: SD VN 43: SD VH
gnss_acc = novatel(:,37:43);
gnss_acc = novatel(:,37:39);

no_epochs = length(in_profile(:,1));

% EPH Data
eph = read_GPSbroadcast('brdc0460.20n');

% RTKLIB Processed Data
rtk1 = csvread('dat/rtklib_pos.csv',1,1);
rtk_tp = rtk1(:,1);
rtk_p = rtk1(:,2:4);
gnss_acc = rtk1(:,7:9);

fid = fopen('drive6.pos.stat');
tline = fgetl(fid);
rtk_v = []; rtk_tv = [];
while ischar(tline)
    if contains(tline,'VELACC')
        tmp = split(tline,',');
        rtk_tv = [rtk_tv;str2double(tmp(3))];
        rtk_v = [rtk_v; str2double(tmp(5:7))'];
    end
    tline = fgetl(fid);
end
fclose(fid);
[rtk_t,idx1,idx2] = intersect(rtk_tp,rtk_tv);
rtk_pv = [rtk_t lla2ecef(rtk_p(idx1,:)) rtk_v(idx2,:)];

if 0
fid = fopen('gnss_log_2020_02_14_20_43_20.txt');
tline = fgetl(fid);
% idx = 1; LLA=[];
android_ins = [];
while ischar(tline)
    if contains(tline,'#')
        tline = fgetl(fid);
        continue
    end
    
    if contains(tline,'Ins')
        tmp = split(tline,',');
        time = mod(str2double(tmp{5})/1000 + 86400*4,86400*7);
        ins = str2double(tmp(6:11))';
        android_ins = [android_ins;time ins];
        tline = fgetl(fid);
        continue
    end
    
%     if contains(tline,'Fix,gps')
%         tmp = split(tline,',');
%         LLA(idx,:) = str2double(tmp(3:5))';
%         tline = fgetl(fid);
%         idx = idx+1;
%         continue
%     end
    
    tline =fgetl(fid);
        
end
save('android_ins.mat','android_ins')
else
    load('android_ins.mat')
end

% imu = [android_ins(:,1),rad2deg(android_ins(:,5:7)),android_ins(:,2:4)];

% IMU
imu = csvread('part6_ins_split.csv');
% novatel = novatel(1:20:end,:); % resample

% mode
% 1: Input: NovAtel Truth
% 2: Input: NovAtel IMU & RTKLIB
% 3: Input: NovAtel IMU & NovAtel Raw
% 4: Demo Simulation
% 5: Use NovAtel truth pos/vel and NovAtel INS
mode = 5;
mode = 2;

% Loosely coupled ECEF Inertial navigation and GNSS integrated navigation
% simulation
[out_profile,out_errors,out_IMU_bias_est,out_clock,out_KF_SD,out_gnss,imu_profile,gnss_errors] =...
    Loosely_coupled_INS_GNSS(in_profile,no_epochs,initialization_errors...
    ,IMU_errors,GNSS_config,LC_KF_config,mode,imu,eph,WN,TOW,rtk_pv,gnss_acc,settings);

% Plot the input motion profile and the errors (may not work in Octave).
% close all;
idx = find(out_gnss(:,1)==0);
out_gnss(idx,:) = [];
idx = find(out_profile(:,2)==0);
out_profile(idx,:) = [];

Plot_profile(in_profile);
Plot_profile(out_profile);

figure();hold on
plot(rad2deg(out_profile(:,3)),rad2deg(out_profile(:,2)),'b.')

figure();hold on
plot(rad2deg(in_profile(:,3)),rad2deg(in_profile(:,2)),'b.')

gnss_lla = ecef2lla(out_gnss);
figure()
plot(gnss_lla(:,2),gnss_lla(:,1),'r.')

imu_lla = ecef2lla(imu_profile(:,1:3));
figure()
plot(imu_lla(:,2),imu_lla(:,1),'k.')

figure()
hold on
plot(rad2deg(in_profile(:,3)),rad2deg(in_profile(:,2)),'g.')
plot(gnss_lla(:,2),gnss_lla(:,1),'r.')
plot(rad2deg(out_profile(:,3)),rad2deg(out_profile(:,2)),'b.')
plot(imu_lla(:,2),imu_lla(:,1),'k.')
legend('truth','rtklib gnss','LC gnss/ins')
return
Plot_errors(out_errors);

% Write output profile and errors file
Write_profile(output_profile_name,out_profile);
Write_errors(output_errors_name,out_errors);

% Ends
end

function [out_profile,kf_errors,out_IMU_bias_est,out_clock,out_KF_SD,out_gnss,imu_profile,gnss_errors] =...
    Loosely_coupled_INS_GNSS(in_profile,no_epochs,initialization_errors,...
    IMU_errors,GNSS_config,LC_KF_config,mode,imu,eph,WN,TOW,rtk_pv,gnss_acc,settings)
%Loosely_coupled_INS_GNSS - Simulates inertial navigation using ECEF
% navigation equations and kinematic model, GNSS using a least-squares
% positioning algorithm, and loosely-coupled INS/GNSS integration. 
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
%
% Inputs:
%   in_profile   True motion profile array
%   no_epochs    Number of epochs of profile data
%   initialization_errors
%     .delta_r_eb_n     position error resolved along NED (m)
%     .delta_v_eb_n     velocity error resolved along NED (m/s)
%     .delta_eul_nb_n   attitude error as NED Euler angles (rad)
%   IMU_errors
%     .delta_r_eb_n     position error resolved along NED (m)
%     .b_a              Accelerometer biases (m/s^2)
%     .b_g              Gyro biases (rad/s)
%     .M_a              Accelerometer scale factor and cross coupling errors
%     .M_g              Gyro scale factor and cross coupling errors            
%     .G_g              Gyro g-dependent biases (rad-sec/m)             
%     .accel_noise_root_PSD   Accelerometer noise root PSD (m s^-1.5)
%     .gyro_noise_root_PSD    Gyro noise root PSD (rad s^-0.5)
%     .accel_quant_level      Accelerometer quantization level (m/s^2)
%     .gyro_quant_level       Gyro quantization level (rad/s)
%   GNSS_config
%     .epoch_interval     Interval between GNSS epochs (s)
%     .init_est_r_ea_e    Initial estimated position (m; ECEF)
%     .no_sat             Number of satellites in constellation
%     .r_os               Orbital radius of satellites (m)
%     .inclination        Inclination angle of satellites (deg)
%     .const_delta_lambda Longitude offset of constellation (deg)
%     .const_delta_t      Timing offset of constellation (s)
%     .mask_angle         Mask angle (deg)
%     .SIS_err_SD         Signal in space error SD (m)
%     .zenith_iono_err_SD Zenith ionosphere error SD (m)
%     .zenith_trop_err_SD Zenith troposphere error SD (m)
%     .code_track_err_SD  Code tracking error SD (m)
%     .rate_track_err_SD  Range rate tracking error SD (m/s)
%     .rx_clock_offset    Receiver clock offset at time=0 (m)
%     .rx_clock_drift     Receiver clock drift at time=0 (m/s)
%   LC_KF_config
%     .init_att_unc           Initial attitude uncertainty per axis (rad)
%     .init_vel_unc           Initial velocity uncertainty per axis (m/s)
%     .init_pos_unc           Initial position uncertainty per axis (m)
%     .init_b_a_unc           Initial accel. bias uncertainty (m/s^2)
%     .init_b_g_unc           Initial gyro. bias uncertainty (rad/s)
%     .gyro_noise_PSD         Gyro noise PSD (rad^2/s)
%     .accel_noise_PSD        Accelerometer noise PSD (m^2 s^-3)
%     .accel_bias_PSD         Accelerometer bias random walk PSD (m^2 s^-5)
%     .gyro_bias_PSD          Gyro bias random walk PSD (rad^2 s^-3)
%     .pos_meas_SD            Position measurement noise SD per axis (m)
%     .vel_meas_SD            Velocity measurement noise SD per axis (m/s)
%
% Outputs:
%   out_profile        Navigation solution as a motion profile array
%   out_errors         Navigation solution error array
%   out_IMU_bias_est   Kalman filter IMU bias estimate array
%   out_clock          GNSS Receiver clock estimate array
%   out_KF_SD          Output Kalman filter state uncertainties
%
% Format of motion profiles:
%  Column 1: time (sec)
%  Column 2: latitude (rad)
%  Column 3: longitude (rad)
%  Column 4: height (m)
%  Column 5: north velocity (m/s)
%  Column 6: east velocity (m/s)
%  Column 7: down velocity (m/s)
%  Column 8: roll angle of body w.r.t NED (rad)
%  Column 9: pitch angle of body w.r.t NED (rad)
%  Column 10: yaw angle of body w.r.t NED (rad)
%
% Format of error array:
%  Column 1: time (sec)
%  Column 2: north position error (m)
%  Column 3: east position error (m)
%  Column 4: down position error (m)
%  Column 5: north velocity error (m/s)
%  Column 6: east velocity error (m/s)
%  Column 7: down velocity (error m/s)
%  Column 8: attitude error about north (rad)
%  Column 9: attitude error about east (rad)
%  Column 10: attitude error about down = heading error  (rad)
%
% Format of output IMU biases array:
%  Column 1: time (sec)
%  Column 2: estimated X accelerometer bias (m/s^2)
%  Column 3: estimated Y accelerometer bias (m/s^2)
%  Column 4: estimated Z accelerometer bias (m/s^2)
%  Column 5: estimated X gyro bias (rad/s)
%  Column 6: estimated Y gyro bias (rad/s)
%  Column 7: estimated Z gyro bias (rad/s)
%
% Format of receiver clock array:
%  Column 1: time (sec)
%  Column 2: estimated clock offset (m)
%  Column 3: estimated clock drift (m/s)
%
% Format of KF state uncertainties array:
%  Column 1: time (sec)
%  Column 2: X attitude error uncertainty (rad)
%  Column 3: Y attitude error uncertainty (rad)
%  Column 4: Z attitude error uncertainty (rad)
%  Column 5: X velocity error uncertainty (m/s)
%  Column 6: Y velocity error uncertainty (m/s)
%  Column 7: Z velocity error uncertainty (m/s)
%  Column 8: X position error uncertainty (m)
%  Column 9: Y position error uncertainty (m)
%  Column 10: Z position error uncertainty (m)
%  Column 11: X accelerometer bias uncertainty (m/s^2)
%  Column 12: Y accelerometer bias uncertainty (m/s^2)
%  Column 13: Z accelerometer bias uncertainty (m/s^2)
%  Column 14: X gyro bias uncertainty (rad/s)
%  Column 15: Y gyro bias uncertainty (rad/s)
%  Column 16: Z gyro bias uncertainty (rad/s)
%  Column 17: clock offset uncertainty (m)
%  Column 18: clock drift uncertainty (m/s)

% Initialize
out_gnss = [];
imu_profile = nan(length(imu(:,1)),6);
% Begins


%% Check Simulation settings
%{
% Initialize the GNSS biases. Note that these are assumed constant throughout 
% the simulation and are based on the initial elevation angles. Therefore, 
% this function is unsuited to simulations longer than about 30 min.
GNSS_biases = Initialize_GNSS_biases(sat_r_es_e,old_true_r_eb_e,true_L_b,...
    true_lambda_b,GNSS_config);

% Generate GNSS measurements
[GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(old_time,...
    sat_r_es_e,sat_v_es_e,old_true_r_eb_e,true_L_b,true_lambda_b,...
    old_true_v_eb_e,GNSS_biases,GNSS_config);
%}

%% Read GNSS RINEX file
%{
dat = read_rinex_obs7('drive6_RX2.20O');
for ii=1:length(dat.data(:,1))
    idx = find(dat.data(:,3) == dat.data(ii,3));
    if idx(1) == ii
        dat.data(ii,12) =0;
    else
        jj = find(idx == ii);
        dat.data(ii,12) = dat.data(idx(jj),5)-dat.data(idx(jj-1),5);
    end
end

ts = unique(dat.data(:,2));
idx = find(dat.data(:,2)==ts(1));
xs = [];
xdots = [];
bsvs = [];
relsvs = [];
for ii = 1:length(idx)
    [~, x, bsv, relsv, xdot] = broadcast_eph2pos_etc(eph,[WN TOW(1)],dat.data(idx(ii),3));
    xs(ii,:)=x;
    xdots(ii,:)=xdot;
    bsvs(ii,:) = bsv;
    relsvs(ii,:) = relsv;
end

if strcmp(settings.velmode,'doppler')
    GNSS_measurements = [dat.data(idx,[4,6]) xs xdots];
    GNSS_measurements(:,2) = GNSS_measurements(:,2)*settings.c/settings.f1;
elseif strcmp(settings.velmode,'carrier')
    GNSS_measurements = [dat.data(idx,[4,12]) xs xdots];
    GNSS_measurements(:,2) = GNSS_measurements(:,2)*settings.c/settings.f1;
end
no_GNSS_meas = length(GNSS_measurements(:,1));
S = dat.data(idx,7);

for ii=1:length(no_GNSS_meas)
    if (dat.data(idx(ii),4) == 0) || (dat.data(idx(ii),8) == 0)
        continue
    else
        [PRIF,~,~] = ionocorr(dat.data(idx(ii),4),settings.f1,dat.data(idx(ii),8),settings.f2);
        GNSS_measurements(ii,1) = PRIF;
    end
end

GNSS_measurements(:,1) = GNSS_measurements(:,1) + bsvs + relsvs;

% satPosLast = xs;
satPosLast = [];
% Determine Least-squares GNSS position solution and use to initialize INS
if sum(mode == [1,3,4])
[GNSS_r_eb_e,GNSS_v_eb_e,est_clock] = GNSS_LS_position_velocity(...
    GNSS_measurements,no_GNSS_meas,GNSS_config.init_est_r_ea_e,[0;0;0],bsvs,relsvs,settings,satPosLast,S);
%}
    
%% Source of GNSS PVT
if mode == 5 % NovAtel Truth
    gnss_time = in_profile(:,1);
    GNSS_r_eb_e = lla2ecef([rad2deg(in_profile(1,2)),rad2deg(in_profile(1,3)),in_profile(1,4)])';
%     GNSS_v_eb_e = ([(in_profile(1,5)),(in_profile(1,6)),in_profile(1,7)])';
    GNSS_v_eb_e = ([(in_profile(1,11)),(in_profile(1,12)),in_profile(1,13)])';
    est_clock = [0 0];
else % RTKLIB
    
    % INS should start before GNSS
    while rtk_pv(1,1) < imu(1,1)
        rtk_pv(1,:) = [];
%         gnss_acc(1,:) = [];
    end
    
    gnss_time = rtk_pv(:,1);
    GNSS_r_eb_e = rtk_pv(1,2:4)';
    GNSS_v_eb_e = rtk_pv(1,5:7)';
    est_clock = [0 0];
end

old_time = gnss_time(1);
no_epochs = length(gnss_time);

old_est_r_eb_e = GNSS_r_eb_e;
old_est_v_eb_e = GNSS_v_eb_e;
[old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n] =...
    pv_ECEF_to_NED(old_est_r_eb_e,old_est_v_eb_e);
est_L_b = old_est_L_b;

out_gnss(1,:) = old_est_r_eb_e';

% Initialize estimated attitude solution
initialization_errors.delta_eul_nb_n = [0;0;0];
% initialization_errors.delta_eul_nb_n = deg2rad([0.986546;	-1.151449; 261.68915]);

% old_est_C_b_n = Initialize_NED_attitude(true_C_b_n,initialization_errors);
old_est_C_b_n = Euler_to_CTM(initialization_errors.delta_eul_nb_n);
[temp1,temp2,old_est_C_b_e] = NED_to_ECEF(old_est_L_b,...
    old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n);

% Initialize output profile record and errors record
out_profile = zeros(no_epochs,10);
kf_errors = zeros(no_epochs,10);
gnss_errors = zeros(no_epochs,10);

% Generate output profile record
out_profile(1,1) = old_time;
out_profile(1,2) = old_est_L_b;
out_profile(1,3) = old_est_lambda_b;
out_profile(1,4) = old_est_h_b;
out_profile(1,5:7) = old_est_v_eb_n';
out_profile(1,8:10) = CTM_to_Euler(old_est_C_b_n')';

% Determine errors and generate output record
% [delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
%     old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n,...
%     true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);

% out_errors(1,1) = old_time;
% out_errors(1,2:4) = delta_r_eb_n';
% out_errors(1,5:7) = delta_v_eb_n';
% out_errors(1,8:10) = delta_eul_nb_n';

% Initialize Kalman filter P matrix and IMU bias states
P_matrix = Initialize_LC_P_matrix(LC_KF_config);
est_IMU_bias = zeros(6,1);
% est_IMU_bias = [imu(1,5:7)';deg2rad(imu(1,2:4)')]-[0 0 9.81 0 0 0]';
        
% Initialize IMU quantization residuals
quant_residuals = [0;0;0;0;0;0];

% Generate IMU bias and clock output records
out_IMU_bias_est(1,1) = old_time;
out_IMU_bias_est(1,2:7) = est_IMU_bias';
out_clock(1,1) = old_time;
out_clock(1,2:3) = est_clock;

% Generate KF uncertainty record
out_KF_SD(1,1) = old_time;
for i =1:15
    out_KF_SD(1,i+1) = sqrt(P_matrix(i,i));
end % for i

% Initialize GNSS model timing

% Progress bar
dots = '....................';
bars = '||||||||||||||||||||';
rewind = '\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b';
fprintf(strcat('Processing: ',dots));
progress_mark = 0;
progress_epoch = 0;

imu_epoch = 1;
while imu(imu_epoch,1)< gnss_time(1)
    imu_epoch = imu_epoch+1;
    imu_idx_start = imu_epoch;
end
    
% tmp=[];
lastwarn('')
% Main loop
for gnss_epoch = 2:no_epochs

    % Update progress bar
    if (gnss_epoch - progress_epoch) > (no_epochs/20)
        progress_mark = progress_mark + 1;
        progress_epoch = gnss_epoch;
        fprintf(strcat(rewind,bars(1:progress_mark),...
            dots(1:(20 - progress_mark))));
    end % if epoch    
    
    
    % Input data from motion profile
    time = gnss_time(gnss_epoch);
    
    % INS-only KF
    while imu(imu_epoch,1)<=gnss_time(gnss_epoch)
        meas_f_ib_b = imu(imu_epoch,5:7)';
%         meas_f_ib_b = [meas_f_ib_b(3);meas_f_ib_b(1);meas_f_ib_b(2)];

        meas_omega_ib_b = deg2rad(imu(imu_epoch,2:4)'); % novatel uses deg
%         meas_omega_ib_b = [meas_omega_ib_b(3);meas_omega_ib_b(1);meas_omega_ib_b(2)];
        
%         rot = [-1 0 0;0 1 0;0 0 -1];
%         rot = [1 0 0;0 -1 0;0 0 -1];
%         rot = eye(3);
        rot = [0 1 0;1 0 0;0 0 -1]; % novatel
        rot = [0 1 0;1 0 0;0 0 -1]; % android
        
        meas_f_ib_b = rot*meas_f_ib_b;
        meas_omega_ib_b = rot*meas_omega_ib_b;
        
%         tmp = [tmp;meas_f_ib_b' meas_omega_ib_b'];
        meas_f_ib_b = meas_f_ib_b - est_IMU_bias(1:3);
        meas_omega_ib_b = meas_omega_ib_b - est_IMU_bias(4:6);
        
        tor_i = imu(imu_epoch,1) - imu(imu_epoch-1,1);
        
        % Update estimated navigation solution
        [est_r_eb_e,est_v_eb_e,est_C_b_e] = Nav_equations_ECEF(tor_i,...
            old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,meas_f_ib_b,...
            meas_omega_ib_b);
        
        imu_profile(imu_epoch-imu_idx_start+1,:) = [est_r_eb_e',est_v_eb_e'];
        
%         if mod(imu_epoch,100)==0
%             figure(103);hold on
%             LLA = ecef2lla(est_r_eb_e');
%             plot(LLA(end,2),LLA(end,1),'m.','markersize',15)
%         end
        
        imu_epoch = imu_epoch + 1;
        if imu_epoch > length(imu(:,1))
            return
        end
        
        % Reset old values
        old_est_r_eb_e = est_r_eb_e;
        old_est_v_eb_e = est_v_eb_e;
        old_est_C_b_e = est_C_b_e;
        
    end
    
        
%     700: 30sec
    %800: 10sec
%     if gnss_epoch > 800
%         gnss_epoch = gnss_epoch + 1;
%         if gnss_epoch == 1000
%             break
%         end
%         continue
%     end
%     
    % GNSS/INS
    if mode == 5
        GNSS_r_eb_e = lla2ecef([rad2deg(in_profile(gnss_epoch,2)),rad2deg(in_profile(gnss_epoch,3)),in_profile(gnss_epoch,4)])';
%         GNSS_v_eb_e = ([(in_profile(gnss_epoch,6)),(in_profile(gnss_epoch,5)),in_profile(gnss_epoch,7)])';
        GNSS_v_eb_e = ([(in_profile(gnss_epoch,11)),(in_profile(gnss_epoch,12)),in_profile(gnss_epoch,13)])';
        out_gnss(gnss_epoch,:) = GNSS_r_eb_e';
        est_clock = [0 0];
    else
        try
            GNSS_r_eb_e = rtk_pv(gnss_epoch,2:4)';
            GNSS_v_eb_e = rtk_pv(gnss_epoch,5:7)';
            est_clock = [0 0];
            out_gnss(gnss_epoch,:) = GNSS_r_eb_e';
            
        catch
            return
        end
    end
    
%     acc = gnss_acc(gnss_epoch,:);

    tor_s = time - old_time;
    
%     if gnss_epoch < 800
    if gnss_epoch >1
    % Run Integration Kalman filter
    try
    [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
        LC_KF_Epoch(GNSS_r_eb_e,GNSS_v_eb_e,tor_s,est_C_b_e,...
        est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,...
        est_L_b,LC_KF_config);
        old_time = time;
    catch
        1;
    end
    end
    
    
%     if gnss_epoch == 1000
%         kf_errors(1,:) = [];
% kf_errors(998:end,:) = [];
% Plot_errors(kf_errors);
%         break
%     end
    
    % Generate IMU bias and clock output records
    out_IMU_bias_est(gnss_epoch,1) = time;
    out_IMU_bias_est(gnss_epoch,2:7) = est_IMU_bias';
    out_clock(gnss_epoch,1) = time;
    out_clock(gnss_epoch,2:3) = est_clock;

    % Generate KF uncertainty output record
    out_KF_SD(gnss_epoch,1) = time;
    for i =1:15
        out_KF_SD(gnss_epoch,i+1) = sqrt(P_matrix(i,i));
    end % for i

    % Convert navigation solution to NED
    [est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] =...
        ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);

    % Generate output profile record
    out_profile(gnss_epoch,1) = time;
    out_profile(gnss_epoch,2) = est_L_b;
    out_profile(gnss_epoch,3) = est_lambda_b;
    out_profile(gnss_epoch,4) = est_h_b;
    out_profile(gnss_epoch,5:7) = est_v_eb_n';
    out_profile(gnss_epoch,8:10) = CTM_to_Euler(est_C_b_n')';

    % Reset old values
    old_est_r_eb_e = est_r_eb_e;
    old_est_v_eb_e = est_v_eb_e;
    old_est_C_b_e = est_C_b_e;
    
    [~,truth_idx] = min(abs(in_profile(:,1) - time));
    true_L_b = in_profile(truth_idx,2);
    true_lambda_b = in_profile(truth_idx,3);
    true_h_b = in_profile(truth_idx,4);
    true_v_eb_n = in_profile(truth_idx,5:7)';
    true_eul_nb = in_profile(truth_idx,8:10)';
    true_C_b_n = Euler_to_CTM(true_eul_nb)';
    [true_r_eb_e,true_v_eb_e,true_C_b_e] =...
        NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
    
    [delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
        est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n,true_L_b,...
        true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
    
    kf_errors(gnss_epoch,1) = time-kf_errors(2,1);
    kf_errors(gnss_epoch,2:4) = delta_r_eb_n';
    kf_errors(gnss_epoch,5:7) = delta_v_eb_n';
    kf_errors(gnss_epoch,8:10) = delta_eul_nb_n';    
    
    [gnss_L_b,gnss_lambda_b,gnss_h_b,gnss_v_eb_n] =...
    pv_ECEF_to_NED(GNSS_r_eb_e,GNSS_v_eb_e);
    gnss_C_b_n = est_C_b_n;
    
    [delta_r_eb_n_G,delta_v_eb_n_G,delta_eul_nb_n_G] = Calculate_errors_NED(...
        gnss_L_b,gnss_lambda_b,gnss_h_b,gnss_v_eb_n,gnss_C_b_n,true_L_b,...
        true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
    
    gnss_errors(gnss_epoch,1) = time-gnss_errors(2,1);
    gnss_errors(gnss_epoch,2:4) = delta_r_eb_n_G';
    gnss_errors(gnss_epoch,5:7) = delta_v_eb_n_G';
    gnss_errors(gnss_epoch,8:10) = delta_eul_nb_n_G';
    
%     if mod(gnss_epoch,5)==0
%     figure(103);hold on
%     plot(rad2deg(in_profile(gnss_epoch,3)),rad2deg(in_profile(gnss_epoch,2)),'bx','markersize',15)
%     end
    
    if 0
    figure(103);hold on
    
    plot(rad2deg(out_profile(gnss_epoch,3)),rad2deg(out_profile(gnss_epoch,2)),'m.')
    LLA = ecef2lla(out_gnss);
    
    plot(LLA(end,2),LLA(end,1),'r.')
    plot(rad2deg(in_profile(gnss_epoch,3)),rad2deg(in_profile(gnss_epoch,2)),'g.')
%     tmp = ([rad2deg(in_profile(:,2)),rad2deg(in_profile(:,3)),in_profile(:,4)]);
%     figure();plot(tmp(:,2),tmp(:,1))
    end
    
    if ~isreal(out_profile)
        1;
    end
end %epoch

% Complete progress bar
fprintf(strcat(rewind,bars,'\n'));
kf_errors(1,:) = [];
gnss_errors(1,:) = [];
Plot_errors(kf_errors);
Plot_errors(gnss_errors);
%{
figure();plot(kf_errors(:,1),kf_errors(:,2),'b');hold on;plot(gnss_errors(:,1),gnss_errors(:,2),'r');
figure();plot(kf_errors(:,1),kf_errors(:,3),'b');hold on;plot(gnss_errors(:,1),gnss_errors(:,3),'r');
figure();plot(kf_errors(:,1),kf_errors(:,4),'b');hold on;plot(gnss_errors(:,1),gnss_errors(:,4),'r');
figure();plot(kf_errors(:,1),kf_errors(:,5),'b');hold on;plot(gnss_errors(:,1),gnss_errors(:,5),'r');
figure();plot(kf_errors(:,1),kf_errors(:,6),'b');hold on;plot(gnss_errors(:,1),gnss_errors(:,6),'r');
figure();plot(kf_errors(:,1),kf_errors(:,7),'b');hold on;plot(gnss_errors(:,1),gnss_errors(:,7),'r');

figure()
%}
% Ends
end