function varargout = main(varargin)
clc; clear all; close all;
addpath('..\')
addpath('subfunctions')
%INS_GNSS_Demo_7
%SCRIPT Tightly coupled INS/GNSS demo:
%   Profile_1 (60s artificial car motion with two 90 deg turns)
%   Consumer-grade IMU

[settings, IMU_errors, GNSS_config, TC_KF_config] = initSettings_TC();

% 1: simulation, 2: novatel, 3: android
mode = 2;

if mode == 1
% CONFIGURATION
% Input truth motion profile filename
input_profile_name = 'Profile_1.csv';
% Output motion profile and error filenames
output_profile_name = 'INS_GNSS_Demo_7_Profile.csv';
output_errors_name = 'INS_GNSS_Demo_7_Errors.csv';

% Attitude initialization error (deg, converted to rad; @N,E,D)
initialization_errors.delta_eul_nb_n = [-0.5;0.4;2]*settings.deg_to_rad; % rad

% Seeding of the random number generator for reproducability. Change 
% this value for a different random number sequence (may not work in Octave).
% RandStream.Type = (RandStream('mt19937ar','seed',1));
RandStream('mt19937ar','seed',1);

% Input truth motion profile from .csv format file
[in_profile,no_epochs,ok] = Read_profile(input_profile_name);

% End script if there is a problem with the file
if ~ok
    return;
end %if

elseif mode == 2
    novatel = csvread('part6_edit.csv',2,0);
    novatel(:,end-2:end) = deg2rad(novatel(:,end-2:end));
    
    in_profile = [novatel(:,2),deg2rad(novatel(:,3:4)),novatel(:,5),novatel(:,19),novatel(:,18),-novatel(:,20),deg2rad(novatel(:,33)),deg2rad(novatel(:,32)),deg2rad(novatel(:,31)), novatel(:,21:23)];
    gnss_acc = novatel(:,37:43);
    gnss_acc = novatel(:,37:39);

    % Attitude initialization error (deg, converted to rad; @N,E,D)
    initialization_errors.delta_eul_nb_n = [-0.5;0.4;2]*settings.deg_to_rad; % rad

    WN = novatel(1,1);
    TOW = novatel(:,2);
    no_epochs = length(in_profile(:,1));
    
    eph = read_GPSbroadcast('brdc0460.20n');
elseif mode == 3
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
%     load('android_ins.mat')
end

% Tightly coupled ECEF Inertial navigation and GNSS integrated navigation
% simulation
if mode == 1
[out_profile,out_errors,out_IMU_bias_est,out_clock,out_KF_SD,out_gnss,out_kf] =...
    Tightly_coupled_INS_GNSS(in_profile,no_epochs,initialization_errors...
    ,IMU_errors,GNSS_config,TC_KF_config,'sim',novatel,eph,WN,TOW,settings);
elseif mode == 2
[out_profile,out_errors,out_IMU_bias_est,out_clock,out_KF_SD,out_gnss,out_kf] =...
    Tightly_coupled_INS_GNSS(in_profile,no_epochs,initialization_errors...
    ,IMU_errors,GNSS_config,TC_KF_config,'real',novatel,eph,WN,TOW,settings);
end

% Plot the input motion profile and the errors (may not work in Octave).
close all;

Plot_profile(in_profile);
Plot_profile(out_profile);
Plot_errors(out_errors);
%{
% Write output profile and errors file
Write_profile(output_profile_name,out_profile);
Write_errors(output_errors_name,out_errors);


figure()
subplot(3,1,1);hold on
plot(out_gnss(:,1),'r')
plot(out_kf(:,1),'b')
subplot(3,1,2);hold on
plot(out_gnss(:,2),'r')
plot(out_kf(:,2),'b')
subplot(3,1,3);hold on
plot(out_gnss(:,3),'r')
plot(out_kf(:,3),'b')
%}
figure();hold on
plot(rad2deg(out_profile(:,3)),rad2deg(out_profile(:,2)),'b.')
gnss_lla = ecef2lla(out_gnss);
plot(gnss_lla(:,2),gnss_lla(:,1),'r.')
plot(rad2deg(in_profile(:,3)),rad2deg(in_profile(:,2)),'g.')
legend('TC EKF','GNSS LS','Truth')

figure();
truth = lla2ecef(rad2deg(in_profile(:,2:4)));
idx = out_gnss(:,1)~=0;
subplot(3,1,1);hold on
plot(out_gnss(idx,1)-truth(idx,1),'.')
plot(out_kf(idx,1)-truth(idx,1),'.')
ylabel('X(m)')
legend('GNSS Error','TC Error')
subplot(3,1,2);hold on
plot(out_gnss(idx,2)-truth(idx,2),'.')
plot(out_kf(idx,2)-truth(idx,2),'.')
ylabel('Y(m)')
subplot(3,1,3);hold on
plot(out_gnss(idx,3)-truth(idx,3),'.')
plot(out_kf(idx,3)-truth(idx,3),'.')
ylabel('Z(m)')
% Ends
end
