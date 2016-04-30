%% Code to grab data, parse, adjust timing, run the filters, and plot 
% Adam Werries 2016, see Apache 2.0 license.
close all;
addpath('GrovesCode');
addpath('Utilities');
addpath('Tuning');

if ~exist('last_applanix_dir', 'var') || ~ischar(last_applanix_dir) 
   last_applanix_dir = pwd;
end
text_files = {'*.txt;*.csv;*.log','Data files (*.txt,*.csv,*.log)'; '*.*', 'All Files (*.*)'};
%% Import Applanix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ground truth import
disp('Please select Applanix log file')
[applanix_file, applanix_path] = uigetfile(text_files, 'Select Applanix Log File', last_applanix_dir);
last_applanix_dir = applanix_path;

gt = csvread([applanix_path applanix_file]);
gt_t = gt(:,1);
gt_vel = gt(:,8:10);
gt_speed = gt(:,21);
gt_rot = gt(:,11:13);
gt_accel = gt(:,14:16);
gt_xyz = gt(:,27:29);


%% Import NovAtel data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Format:  1     2                   3               4  5  6  7      8      
%         time, sol good? (1 or 0), WAAS? (1 or 0), x, y, z, sig_x, sig_y
%         9      10  11  12  13      14      15      16        17
%         sig_z, vx, vy, vz, sig_vx, sig_vy, sig_vz, num_sats, sol_sats
if ~exist('last_lowcost_dir', 'var') || ~ischar(last_lowcost_dir)
   last_lowcost_dir = last_applanix_dir; 
end
disp('Please select NovAtel log file')
[novatel_file, novatel_path] = uigetfile(text_files, 'Select NovAtel Log File', last_lowcost_dir);
last_lowcost_dir = novatel_path;

novatel = csvread([novatel_path novatel_file]);
nov_time = novatel(:,1);
% Convert ECEF to lat-long-altitude
lla = ecef2lla(novatel(:,4:6));
% Convert latlon to UTM
[gps_x,gps_y,utmzone] = deg2utm(lla(:,1),lla(:,2));
gps_h = -lla(:,3);

%% Import IMU data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Format:  1                2              3        4        5        6       7       8
%         meas_start_time, meas_end_time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
disp('Please select IMU log file')
[imu_file, imu_path] = uigetfile(text_files, 'Select IMU Log File', last_lowcost_dir);
imu = csvread([imu_path imu_file]);
% only using end_time here
imu = imu(:,2:end);
% convert g's to m/s^2 using local gravity estimate
imu(:,2:4) = imu(:,2:4)*9.80097;
% rotate sensor readings 180 deg about the x-axis
rotation = [1   0         0;
            0   cosd(180) -sind(180)
            0   sind(180) cosd(180)];
imu(:,2:4) = (rotation*imu(:,2:4)')';
imu(:,5:7) = (rotation*imu(:,5:7)')';
%% Correct time ranges %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timelag = 0;
[start_time,Is] = max([min(nov_time),min(gt_t),min(imu(:,1))]);
[end_time,Ie] = min([max(nov_time),max(gt_t),max(imu(:,1))]);
end_time = end_time - start_time;
gt_t = gt_t - start_time + timelag;
nov_time = nov_time - start_time;
imu(:,1) = imu(:,1) - start_time;
% cut off extra novatel data at end, fold time back in
nov_mask = nov_time < end_time & nov_time > 0;
nov_time = nov_time(nov_mask);
novatel = novatel(nov_mask,:);
lla = lla(nov_mask,:);
novatel(:,1) = nov_time;
% cut off extra IMU data at end
imu = imu(imu(:,1) < end_time, :);
imu = imu(imu(:,1) > 0, :);
imu_time = imu(:,1);

%% Generate filter time, configurable epochs
epoch = 0.04;
filter_time = 0:epoch:imu_time(end);

%% Generate ground truth

min_x = min([min(gps_x),min(gt_xyz(:,2))]);
min_y = min([min(gps_y),min(gt_xyz(:,1))]);
min_z = min([min(gps_h),min(gt_xyz(:,3))]);
gps_x = gps_x(nov_mask) - min_x;
gps_y = gps_y(nov_mask) - min_y;
gps_h = gps_h(nov_mask);
ground_truth = zeros(length(nov_time), 6);
ground_truth_full = zeros(length(filter_time), 9);

ground_truth(:,1) =  (interp1(gt_t,gt_xyz(:,2),nov_time)-min_x)';
ground_truth(:,2) =  (interp1(gt_t,gt_xyz(:,1),nov_time)-min_y)';
ground_truth(:,3) =  interp1(gt_t,gt_xyz(:,3),nov_time)';
ground_truth(:,4) =  interp1(gt_t,gt_rot(:,1),nov_time)';
ground_truth(:,5) =  interp1(gt_t,gt_rot(:,2),nov_time)';
ground_truth(:,6) =  interp1(gt_t,gt_rot(:,3),nov_time)';

ground_truth_full(:,1) = (interp1(gt_t,gt_xyz(:,2),filter_time)-min_x)';
ground_truth_full(:,2) = (interp1(gt_t,gt_xyz(:,1),filter_time)-min_y)';
ground_truth_full(:,3) = interp1(gt_t,gt_xyz(:,3),filter_time)';
ground_truth_full(:,4) = interp1(gt_t,gt_rot(:,1),filter_time)';
ground_truth_full(:,5) = interp1(gt_t,gt_rot(:,2),filter_time)';
ground_truth_full(:,6) = interp1(gt_t,gt_rot(:,3),filter_time)';
ground_truth_full(:,7) = interp1(gt_t,gt_vel(:,1),filter_time)';
ground_truth_full(:,8) = interp1(gt_t,gt_vel(:,2),filter_time)';
ground_truth_full(:,9) = interp1(gt_t,gt_vel(:,3),filter_time)';

%% Initialize Error Estimates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constants
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;
mug_to_mps2 = 9.80665E-6;

% CONFIGURATION
% Output motion profile and error filenames
output_profile_name = 'Output_Profile.csv';

% Initial attitude uncertainty per axis (deg, converted to rad)
LC_KF_config.init_att_unc = degtorad(1.5);
% Initial velocity uncertainty per axis (m/s)
LC_KF_config.init_vel_unc = 0.05;
% Initial position uncertainty per axis (m)
LC_KF_config.init_pos_unc = 0.2;
% Initial accelerometer bias uncertainty (micro-g, converted
% to m/s^2)
LC_KF_config.init_b_a_unc = 1000 * mug_to_mps2;
% Initial gyro bias uncertainty(deg/hour, converted to rad/sec)
LC_KF_config.init_b_g_unc = 100 * deg_to_rad / 3600;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
LC_KF_config.gyro_noise_PSD = (20 * deg_to_rad / 60)^2;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
LC_KF_config.accel_noise_PSD = (1700 * mug_to_mps2)^2;
% Accelerometer bias random walk PSD (m^2 s^-5)
LC_KF_config.accel_bias_PSD = 1e-10;
% Gyro bias random walk PSD (rad^2 s^-3)
LC_KF_config.gyro_bias_PSD = 1e-10;
% Lever arm from IMU to GPS
LC_KF_config.lever_arm = [0; 0.3046; 0.9209209];
LC_KF_config.gps_correction = [0; -0.1783567; -1.5190381];
% Minimum and maximum R matrix values
LC_KF_config.pos_sd_min = 0.1;
LC_KF_config.pos_sd_max = 50;
LC_KF_config.vel_sd_min = 1;
LC_KF_config.vel_sd_max = 15;
% Initial estimate of accelerometer and gyro static bias
est_IMU_bias = [
    0.0672
    0.2406
    0.8969
   -0.0309
    0.0004
   -0.0143];
% number of measurements to use for innovation adaptive estimation
% LC_KF_config.n = 470;
LC_KF_config.n = Inf;

%% Format initial conditions
% x y z vx vy vz r p y
% init_cond = [novatel(1,4:6) novatel(1,10:12) deg2rad(-40) deg2rad(-8.5) deg2rad(-125)];
init_cond = [novatel(1,4:6) novatel(1,10:12) deg2rad(-13.1) deg2rad(-.455) deg2rad(-140)];

%% Loosely coupled ECEF INS and GNSS integrated navigation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Beginning processing');
% disp(LC_KF_config);
[out_profile,out_IMU_bias_est,out_KF_SD, out_R_matrix, out_Q_matrix, corrections] = ...
    Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, novatel, imu, LC_KF_config, est_IMU_bias);

generate_error_metrics
generate_plots
mean(out_IMU_bias_est(:,2:end))'
