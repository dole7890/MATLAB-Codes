function varargout = main(varargin)
clc; clear all; close all;
addpath('subfunctions')
addpath('data')
% main script

% Constants
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;
micro_g_to_meters_per_second_squared = 9.80665E-6;

% CONFIGURATION
% Input truth motion profile filename
input_profile_name = 'Profile_1.csv';
% Output motion profile and error filenames
output_profile_name = 'INS_GNSS_Demo_3_Profile.csv';
output_errors_name = 'INS_GNSS_Demo_3_Errors.csv';

% Attitude initialization error (deg, converted to rad; @N,E,D)
initialization_errors.delta_eul_nb_n = [-0.05;0.04;1]*deg_to_rad; % rad

% Accelerometer biases (micro-g, converted to m/s^2; body axes)
IMU_errors.b_a = [900;-1300;800] * micro_g_to_meters_per_second_squared;
% Gyro biases (deg/hour, converted to rad/sec; body axes)
IMU_errors.b_g = [-9;13;-8] * deg_to_rad / 3600;
% Accelerometer scale factor and cross coupling errors (ppm, converted to
% unitless; body axes)
IMU_errors.M_a = [500, -300, 200;...
                 -150, -600, 250;...
                 -250,  100, 450] * 1E-6;
% Gyro scale factor and cross coupling errors (ppm, converted to unitless;
% body axes)
IMU_errors.M_g = [400, -300,  250;...
                    0, -300, -150;...
                    0,    0, -350] * 1E-6;             
% Gyro g-dependent biases (deg/hour/g, converted to rad-sec/m; body axes)
IMU_errors.G_g = [0.9, -1.1, -0.6;...
                 -0.5,  1.9, -1.6;...
                  0.3,  1.1, -1.3] * deg_to_rad / (3600 * 9.80665);             
% Accelerometer noise root PSD (micro-g per root Hz, converted to m s^-1.5)                
IMU_errors.accel_noise_root_PSD = 100 *...
    micro_g_to_meters_per_second_squared;
% Gyro noise root PSD (deg per root hour, converted to rad s^-0.5)                
IMU_errors.gyro_noise_root_PSD = 0.01 * deg_to_rad / 60;
% Accelerometer quantization level (m/s^2)
IMU_errors.accel_quant_level = 1E-2;
% Gyro quantization level (rad/s)
IMU_errors.gyro_quant_level = 2E-4;

% Interval between GNSS epochs (s)
GNSS_config.epoch_interval = 0.5;

% Initial estimated position (m; ECEF)
GNSS_config.init_est_r_ea_e = [0;0;0];

% Number of satellites in constellation
GNSS_config.no_sat = 30;
% Orbital radius of satellites (m)
GNSS_config.r_os = 2.656175E7;
% Inclination angle of satellites (deg)
GNSS_config.inclination = 55;
% Longitude offset of constellation (deg)
GNSS_config.const_delta_lambda = 0;
% Timing offset of constellation (s)
GNSS_config.const_delta_t = 0;

% Mask angle (deg)
GNSS_config.mask_angle = 10;
% Signal in space error SD (m) *Give residual where corrections are applied
GNSS_config.SIS_err_SD = 1;
% Zenith ionosphere error SD (m) *Give residual where corrections are applied
GNSS_config.zenith_iono_err_SD = 2;
% Zenith troposphere error SD (m) *Give residual where corrections are applied
GNSS_config.zenith_trop_err_SD = 0.2;
% Code tracking error SD (m) *Can extend to account for multipath
GNSS_config.code_track_err_SD = 1;
% Range rate tracking error SD (m/s) *Can extend to account for multipath
GNSS_config.rate_track_err_SD = 0.02;
% Receiver clock offset at time=0 (m);
GNSS_config.rx_clock_offset = 10000;
% Receiver clock drift at time=0 (m/s);
GNSS_config.rx_clock_drift = 100;

% Initial attitude uncertainty per axis (deg, converted to rad)
LC_KF_config.init_att_unc = degtorad(1);
% Initial velocity uncertainty per axis (m/s)
LC_KF_config.init_vel_unc = 0.1;
% Initial position uncertainty per axis (m)
LC_KF_config.init_pos_unc = 10;
% Initial accelerometer bias uncertainty per instrument (micro-g, converted
% to m/s^2)
LC_KF_config.init_b_a_unc = 1000 * micro_g_to_meters_per_second_squared;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
LC_KF_config.init_b_g_unc = 10 * deg_to_rad / 3600;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
LC_KF_config.gyro_noise_PSD = (0.02 * deg_to_rad / 60)^2;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
LC_KF_config.accel_noise_PSD = (200 *...
    micro_g_to_meters_per_second_squared)^2;
% Accelerometer bias random walk PSD (m^2 s^-5)
LC_KF_config.accel_bias_PSD = 1.0E-7;
% Gyro bias random walk PSD (rad^2 s^-3)
LC_KF_config.gyro_bias_PSD = 2.0E-12;

% Position measurement noise SD per axis (m)
LC_KF_config.pos_meas_SD = 2.5;
% Velocity measurement noise SD per axis (m/s)
LC_KF_config.vel_meas_SD = 0.1;

% Seeding of the random number generator for reproducability. Change 
% this value for a different random number sequence (may not work in Octave).
RandStream.setGlobalStream(RandStream('mt19937ar','seed',1));

% Begins

% Input truth motion profile from .csv format file
[in_profile,no_epochs,ok] = Read_profile(input_profile_name);

% End script if there is a problem with the file
if ~ok
    return;
end %if

%%

% Tightly coupled ECEF Inertial navigation and GNSS integrated navigation
% simulation
[out_profile,out_errors,out_IMU_bias_est,out_clock,out_KF_SD] =...
    Loosely_coupled_INS_GNSS(in_profile,no_epochs,initialization_errors...
    ,IMU_errors,GNSS_config,LC_KF_config);

%%

% Plot the input motion profile and the errors (may not work in Octave).
close all;
Plot_profile(in_profile);
Plot_errors(out_errors);

% Write output profile and errors file
Write_profile(output_profile_name,out_profile);
Write_errors(output_errors_name,out_errors);

end




function [out_profile,out_errors,out_IMU_bias_est,out_clock,out_KF_SD] =...
    Loosely_coupled_INS_GNSS(in_profile,no_epochs,initialization_errors,...
    IMU_errors,GNSS_config,LC_KF_config)
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

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Begins

imu = csvread('part6_edit.csv',2,0);
imu(:,end-2:end) = deg2rad(imu(:,end-2:end));
eph = read_GPSbroadcast('brdc0460.20n');
WN = 2092;
TOW = 531813.2:0.2:534238.6;
t_input = [WN TOW(1)];
% sat_r = nan(32,1);
% sat_v = nan(32,1);
sat_r = [];
sat_v = [];
for prn=1:32
    [health, sat_r{prn}, bsv, relsv, sat_v{prn}] = broadcast_eph2pos_etc(eph,t_input,prn);
end


% Initialize true navigation solution
old_time = TOW(1);
true_L_b = in_profile(1,2);
true_lambda_b = in_profile(1,3);
true_h_b = in_profile(1,4);
true_v_eb_n = in_profile(1,5:7)';
true_eul_nb = in_profile(1,8:10)';
true_C_b_n = Euler_to_CTM(true_eul_nb)';
[old_true_r_eb_e,old_true_v_eb_e,old_true_C_b_e] =...
    NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);

% Determine satellite positions and velocities
[sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(old_time,...
    GNSS_config);


% Initialize the GNSS biases. Note that these are assumed constant throughout 
% the simulation and are based on the initial elevation angles. Therefore, 
% this function is unsuited to simulations longer than about 30 min.
GNSS_biases = Initialize_GNSS_biases(sat_r_es_e,old_true_r_eb_e,true_L_b,...
    true_lambda_b,GNSS_config);

% Generate GNSS measurements
% [GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(old_time,...
%     sat_r_es_e,sat_v_es_e,old_true_r_eb_e,true_L_b,true_lambda_b,...
%     old_true_v_eb_e,GNSS_biases,GNSS_config);

%%
if 0
    dat = read_rinex_obs7('drive6_RX2.20O');
    save('temp.mat','dat')
else
    load('temp.mat')
end
ts = unique(dat.data(:,2));
idx = find(dat.data(:,2)==ts(1));
xs = [];
xdots = [];
for ii = 1:length(idx)
    [~, x, ~, ~, xdot] = broadcast_eph2pos_etc(eph,[WN TOW(1)],dat.data(ii,3));
    xs(ii,:)=x;
    xdots(ii,:)=xdot;
end
GNSS_measurements = [dat.data(idx,[4,6]) xs xdots];
no_GNSS_meas = length(GNSS_measurements(:,1));
%%

% Determine Least-squares GNSS position solution and use to initialize INS
[GNSS_r_eb_e,GNSS_v_eb_e,est_clock] = GNSS_LS_position_velocity(...
    GNSS_measurements,no_GNSS_meas,GNSS_config.init_est_r_ea_e,[0;0;0]);
old_est_r_eb_e = GNSS_r_eb_e;
old_est_v_eb_e = GNSS_v_eb_e;
[old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n] =...
    pv_ECEF_to_NED(old_est_r_eb_e,old_est_v_eb_e);
est_L_b = old_est_L_b;

%%
% [GNSS_r,GNSS_v,b] = GNSS_LS_position_velocity(...
%     OBS,n_OBS,GNSS_config.init_est_r_ea_e,[0;0;0]);
% old_r = GNSS_r;
% old_v = GNSS_v;
% [old_est_L_b2,old_est_lambda_b2,old_est_h_b2,old_est_v_eb_n2] =...
%     pv_ECEF_to_NED(old_r,old_v);
% est_L_b2 = old_est_L_b2;
% % Initialize estimated attitude solution
% old_est_C_b_n2 = Initialize_NED_attitude(true_C_b_n,initialization_errors);
% [temp1,temp2,old_est_C_b_e] = NED_to_ECEF(old_est_L_b2,...
%     old_est_lambda_b2,old_est_h_b2,old_est_v_eb_n2,old_est_C_b_n2);

%%

% Initialize estimated attitude solution
old_est_C_b_n = Initialize_NED_attitude(true_C_b_n,initialization_errors);
[temp1,temp2,old_est_C_b_e] = NED_to_ECEF(old_est_L_b,...
    old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n);

% Initialize output profile record and errors record
no_epochs = length(TOW);
out_profile = zeros(no_epochs,10);
out_errors = zeros(no_epochs,10);

% Generate output profile record
out_profile(1,1) = old_time;
out_profile(1,2) = old_est_L_b;
out_profile(1,3) = old_est_lambda_b;
out_profile(1,4) = old_est_h_b;
out_profile(1,5:7) = old_est_v_eb_n';
out_profile(1,8:10) = CTM_to_Euler(old_est_C_b_n')';

% Determine errors and generate output record
[delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
    old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n,...
    true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
out_errors(1,1) = old_time;
out_errors(1,2:4) = delta_r_eb_n';
out_errors(1,5:7) = delta_v_eb_n';
out_errors(1,8:10) = delta_eul_nb_n';

% Initialize Kalman filter P matrix and IMU bias states
P_matrix = Initialize_LC_P_matrix(LC_KF_config);
est_IMU_bias = zeros(6,1);

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
time_last_GNSS = old_time;
GNSS_epoch = 1;

% Progress bar
dots = '....................';
bars = '||||||||||||||||||||';
rewind = '\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b';
fprintf(strcat('Processing: ',dots));
progress_mark = 0;
progress_epoch = 0;

% Main loop
for epoch = 2:no_epochs

    % Update progress bar
    if (epoch - progress_epoch) > (no_epochs/20)
        progress_mark = progress_mark + 1;
        progress_epoch = epoch;
        fprintf(strcat(rewind,bars(1:progress_mark),...
            dots(1:(20 - progress_mark))));
    end % if epoch    
    
    % Input data from motion profile
    time = TOW(epoch);
    true_L_b = in_profile(epoch,2);
    true_lambda_b = in_profile(epoch,3);
    true_h_b = in_profile(epoch,4);
    true_v_eb_n = in_profile(epoch,5:7)';
    true_eul_nb = in_profile(epoch,8:10)';
    true_C_b_n = Euler_to_CTM(true_eul_nb)';
    [true_r_eb_e,true_v_eb_e,true_C_b_e] =...
        NED_to_ECEF(true_L_b,true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);

    % Time interval
    tor_i = time - old_time;
    
    % Calculate specific force and angular rate
    [true_f_ib_b,true_omega_ib_b] = Kinematics_ECEF(tor_i,true_C_b_e,...
        old_true_C_b_e,true_v_eb_e,old_true_v_eb_e,old_true_r_eb_e);
        
     % Simulate IMU errors
    [meas_f_ib_b,meas_omega_ib_b,quant_residuals] = IMU_model(tor_i,...
        true_f_ib_b,true_omega_ib_b,IMU_errors,quant_residuals);
    
    % Correct IMU errors
    meas_f_ib_b = meas_f_ib_b - est_IMU_bias(1:3);
    meas_omega_ib_b = meas_omega_ib_b - est_IMU_bias(4:6);
    
    
    meas_f_ib_b = imu(epoch,25:27)';
    meas_omega_ib_b = imu(epoch,end-2:end)';
    
    % Update estimated navigation solution
    [est_r_eb_e,est_v_eb_e,est_C_b_e] = Nav_equations_ECEF(tor_i,...
        old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,meas_f_ib_b,...
        meas_omega_ib_b);

    % Determine whether to update GNSS simulation and run Kalman filter
    if (time - time_last_GNSS) >= GNSS_config.epoch_interval
        GNSS_epoch = GNSS_epoch + 1;
        tor_s = time - time_last_GNSS;  % KF time interval
        time_last_GNSS = time;
   
        % Determine satellite positions and velocities
        [sat_r_es_e,sat_v_es_e] = Satellite_positions_and_velocities(time,...
            GNSS_config);

        % Generate GNSS measurements
%         [GNSS_measurements,no_GNSS_meas] = Generate_GNSS_measurements(...
%             time,sat_r_es_e,sat_v_es_e,true_r_eb_e,true_L_b,true_lambda_b,...
%             true_v_eb_e,GNSS_biases,GNSS_config);
        
        %%
        ts = unique(dat.data(:,2));
        idx = find(dat.data(:,2)==ts(epoch));
        xs = [];
        xdots = [];
        for ii = 1:length(idx)
            [~, x, ~, ~, xdot] = broadcast_eph2pos_etc(eph,[WN TOW(1)],dat.data(ii,3));
            xs(ii,:)=x;
            xdots(ii,:)=xdot;
        end
        GNSS_measurements = [dat.data(idx,[4,6]) xs xdots];
        no_GNSS_meas = length(GNSS_measurements(:,1));
        
        %%
        
        % Determine GNSS position solution
        [GNSS_r_eb_e,GNSS_v_eb_e,est_clock] = GNSS_LS_position_velocity(...
            GNSS_measurements,no_GNSS_meas,GNSS_r_eb_e,GNSS_v_eb_e);

        % Run Integration Kalman filter
        [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
            LC_KF_Epoch(GNSS_r_eb_e,GNSS_v_eb_e,tor_s,est_C_b_e,...
            est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,...
            est_L_b,LC_KF_config);

        % Generate IMU bias and clock output records
        out_IMU_bias_est(GNSS_epoch,1) = time;
        out_IMU_bias_est(GNSS_epoch,2:7) = est_IMU_bias';
        out_clock(GNSS_epoch,1) = time;
        out_clock(GNSS_epoch,2:3) = est_clock;

        % Generate KF uncertainty output record
        out_KF_SD(GNSS_epoch,1) = time;
        for i =1:15
            out_KF_SD(GNSS_epoch,i+1) = sqrt(P_matrix(i,i));
        end % for i

    end % if time    
    
    % Convert navigation solution to NED
    [est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n] =...
        ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);

    % Generate output profile record
    out_profile(epoch,1) = time;
    out_profile(epoch,2) = est_L_b;
    out_profile(epoch,3) = est_lambda_b;
    out_profile(epoch,4) = est_h_b;
    out_profile(epoch,5:7) = est_v_eb_n';
    out_profile(epoch,8:10) = CTM_to_Euler(est_C_b_n')';
    
    % Determine errors and generate output record
    [delta_r_eb_n,delta_v_eb_n,delta_eul_nb_n] = Calculate_errors_NED(...
        est_L_b,est_lambda_b,est_h_b,est_v_eb_n,est_C_b_n,true_L_b,...
        true_lambda_b,true_h_b,true_v_eb_n,true_C_b_n);
    out_errors(epoch,1) = time;
    out_errors(epoch,2:4) = delta_r_eb_n';
    out_errors(epoch,5:7) = delta_v_eb_n';
    out_errors(epoch,8:10) = delta_eul_nb_n';
    
    % Reset old values
    old_time = time;
    old_true_r_eb_e = true_r_eb_e;
    old_true_v_eb_e = true_v_eb_e;
    old_true_C_b_e = true_C_b_e;
    old_est_r_eb_e = est_r_eb_e;
    old_est_v_eb_e = est_v_eb_e;
    old_est_C_b_e = est_C_b_e;

end %epoch

% Complete progress bar
fprintf(strcat(rewind,bars,'\n'));

% Ends
end