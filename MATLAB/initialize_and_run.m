% Code to grab data, parse, adjust timing, run the filters, and plot 
% Adam Werries 2016, see Apache 2.0 license.
close all; 
clear all;
addpath('GrovesCode');
text_files = {'*.txt;*.csv;*.log','Data files (*.txt,*.csv,*.log)'; '*.*', 'All Files (*.*)'};
%% Import Applanix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ground truth import
disp('Please select Applanix log file')
[applanix_file, applanix_path] = uigetfile(text_files, 'Select Applanix Log File');
gt = csvread([applanix_path applanix_file]);
gt_t = gt(:,1);
gt_vel = gt(:,8:10);
gt_speed = gt(:,21);
gt_rot = gt(:,11:13);
gt_accel = gt(:,14:16);
gt_xyz = gt(:,27:29);
figurec;
plot(gt_xyz(:,2),gt_xyz(:,1));
pathsplit = strsplit(applanix_path, filesep);
title(sprintf('Applanix data: %s%s%s',cell2mat(pathsplit(end-1)),filesep,applanix_file),'Interpreter','none');

%% Import NovAtel data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Format:  1     2                   3               4  5  6  7      8      
%         time, sol good? (1 or 0), WAAS? (1 or 0), x, y, z, sig_x, sig_y
%         9      10  11  12  13      14      15      16        17
%         sig_z, vx, vy, vz, sig_vx, sig_vy, sig_vz, num_sats, sol_sats
disp('Please select NovAtel log file')
[novatel_file, novatel_path] = uigetfile(text_files, 'Select NovAtel Log File', applanix_path);
novatel = csvread([novatel_path novatel_file]);
nov_time = novatel(:,1);
% Convert ECEF to lat-long-altitude
lla = ecef2lla(novatel(:,4:6));
% Convert latlon to UTM
[gps_x,gps_y,utmzone] = deg2utm(lla(:,1),lla(:,2));
gps_h = -lla(:,3);
figurec;
plot(gps_x, gps_y);
pathsplit = strsplit(novatel_path, filesep);
title(sprintf('NovAtel XY data: %s%s%s',cell2mat(pathsplit(end-1)),filesep,novatel_file),'Interpreter','none');

%% Import IMU2 data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Format:  1     2        3        4        5       6       7
%         time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
disp('Please select IMU log file')
[imu_file, imu_path] = uigetfile(text_files, 'Select IMU Log File', novatel_path);
imu = csvread([imu_path imu_file]);
% special correction: the first 48 values in oakland_oct24_meh3 seem to be old serial
% data recorded all at once.
imu = imu(49:end,:);
% Correct accel (must swap axis and convert to m/s^2)
imu(:,2:4) = [-imu(:,3) imu(:,2) -imu(:,4)]/256*9.80665;
% Convert gyro to rad/s
imu(:,5:7) = [-imu(:,5) imu(:,6) -imu(:,7)]*0.001214225560616;

%% Correct time ranges %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[start_time,Is] = max([min(nov_time),min(gt_t),min(imu(:,1))]);
[end_time,Ie] = min([max(nov_time),max(gt_t),max(imu(:,1))]);
end_time = end_time - start_time;
gt_t = gt_t - start_time;
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
epoch = .02;
filter_time = 0:epoch:imu_time(end);

%% Generate ground truth
min_x = min([min(gps_x),min(gt_xyz(:,2))]);
min_y = min([min(gps_y),min(gt_xyz(:,1))]);
min_z = min([min(gps_h),min(gt_xyz(:,3))]);
gps_x = gps_x(nov_mask) - min_x;
gps_y = gps_y(nov_mask) - min_y;
gps_h = gps_h(nov_mask)-1.5;
ground_truth = zeros(length(nov_time), 6);
ground_truth_full = zeros(length(filter_time), 6);

ground_truth(:,1) =  (interp1(gt_t,gt_xyz(:,2),nov_time)-min_x)';
ground_truth(:,2) =  (interp1(gt_t,gt_xyz(:,1),nov_time)-min_y)';
ground_truth(:,3) =  interp1(gt_t,gt_xyz(:,3),nov_time)';
ground_truth(:,4) =  interp1(gt_t,gt_rot(:,1),nov_time)';
ground_truth(:,5) =  interp1(gt_t,gt_rot(:,2),nov_time)';
ground_truth(:,6) =  interp1(gt_t,gt_rot(:,3),nov_time)';

ground_truth_full(:,1) = (interp1(gt_t,gt_xyz(:,2),filter_time)-min_x)';
ground_truth_full(:,2) = (interp1(gt_t,gt_xyz(:,1),filter_time)-min_y)';
ground_truth_full(:,3) = interp1(gt_t,gt_xyz(:,3),filter_time)';
ground_truth_full(:,5) = interp1(gt_t,gt_rot(:,1),filter_time)';
ground_truth_full(:,6) = interp1(gt_t,gt_rot(:,2),filter_time)';
ground_truth_full(:,7) = interp1(gt_t,gt_rot(:,3),filter_time)';


%% Compare ground truth to raw NovAtel data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figurec;
plot(gps_x, gps_y,'r--', 'LineWidth', 2);
xlabel('Easting (m)');
ylabel('Northing (m)');
hold on;
plot(ground_truth(:,1),ground_truth(:,2),'c');
legend('NovAtel','Applanix');
axis equal;

figurec;
subplot(311);
plot(nov_time, gps_x,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting (m)');
hold on; plot(nov_time, ground_truth(:,1),'c');
legend('NovAtel','Applanix');

subplot(312);
plot(nov_time, gps_y,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing (m)');
hold on; plot(nov_time, ground_truth(:,2),'c');
legend('NovAtel','Applanix','Location','SouthEast');

subplot(313);
plot(nov_time, gps_h,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing (m)');
hold on; plot(nov_time, ground_truth(:,3),'c');
legend('NovAtel','Applanix','Location','SouthEast');

%% Initialize Error Estimates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constants
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;
mug_to_mps2 = 9.80665E-6;

% CONFIGURATION
% Output motion profile and error filenames
output_profile_name = 'Output_Profile.csv';

% Initial attitude uncertainty per axis (deg, converted to rad)
LC_KF_config.init_att_unc = degtorad(3);
% Initial velocity uncertainty per axis (m/s)
LC_KF_config.init_vel_unc = 6.2;
% Initial position uncertainty per axis (m)
LC_KF_config.init_pos_unc = 5;
% Initial accelerometer bias uncertainty per instrument (micro-g, converted
% to m/s^2)
LC_KF_config.init_b_a_unc = 2000 * mug_to_mps2;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
LC_KF_config.init_b_g_unc = 20 * deg_to_rad / 3600;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
LC_KF_config.gyro_noise_PSD = (3.6 * deg_to_rad / 60)^2;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
LC_KF_config.accel_noise_PSD = (1050 * mug_to_mps2)^2;
% Accelerometer bias random walk PSD (m^2 s^-5)
LC_KF_config.accel_bias_PSD = 1.0E-5;
% Gyro bias random walk PSD (rad^2 s^-3)
LC_KF_config.gyro_bias_PSD = 2.0E-10;

% Position measurement noise SD per axis (m)
LC_KF_config.pos_meas_SD = 5.2;
% Velocity measurement noise SD per axis (m/s)
LC_KF_config.vel_meas_SD = 0.3;
% number of measurements to use for innovation adaptive estimation
n = Inf;
% Seeding of the random number generator for reproducability. Change 
% this value for a different random number sequence (may not work in Octave).
RandStream.setGlobalStream(RandStream('mt19937ar','seed',1));
%% Format initial conditions
% x y z vx vy vz r p y
init_cond = [novatel(1,4:6) novatel(1,10:12) ground_truth(1,4:6)+[deg2rad(4.2) deg2rad(77.5) deg2rad(-51.5)]];

%% Loosely coupled ECEF INS and GNSS integrated navigation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[out_profile,out_IMU_bias_est,out_KF_SD] = ...
    Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, novatel, imu, LC_KF_config, n);
xyz = out_profile(:,2:4);
llh = ecef2lla(xyz);
[x,y] = deg2utm(llh(:,1),llh(:,2));
x = x-min_x;
y = y-min_y;
h = -llh(:,3)-1.5;
figurec;
plot(x, y,'r--', 'LineWidth', 2);
xlabel('Easting (m)');
ylabel('Northing (m)');
hold on;
plot(ground_truth(:,1),ground_truth(:,2),'c');
plot(gps_x, gps_y,'b-.', 'LineWidth', 2);
legend('Filtered','Applanix','NovAtel');
axis equal;

figurec;
subplot(211);
plot(filter_time, x,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting (m)');
hold on; plot(filter_time, ground_truth_full(:,1),'c');
legend('Filtered','Applanix');

subplot(212);
plot(filter_time, y,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing (m)');
hold on; plot(filter_time, ground_truth_full(:,2),'c');
legend('Filtered','Applanix','Location','SouthEast');

% subplot(313);
% plot(imu_time, h,'r--', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Height (m)');
% hold on; plot(imu_time, ground_truth_full(:,3),'c');
% legend('Filtered','Applanix','Location','SouthEast');
%% Compute error statistics on groundtruth vs NovAtel %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
distance = ((ground_truth(:,1)-gps_x).^2 + (ground_truth(:,2)-gps_y).^2).^0.5;
fprintf('\nRMS Raw:%f\n',rms(distance))
fprintf('Max Raw:%f\n',max(distance))

%% Compute error statistics on groundtruth vs filter output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
fprintf('\nRMS Filter:%f\n',rms(distance))
fprintf('Max Filter:%f\n',max(distance))
figurec;
subplot(211);
plot(filter_time, x-ground_truth_full(:,1),'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting error (m)');
ylim([-10 10])
subplot(212);
plot(filter_time, y-ground_truth_full(:,2),'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing error (m)');
ylim([-10 10])



