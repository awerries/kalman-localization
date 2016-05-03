%% Code to grab data, parse, adjust timing
% Adam Werries 2016, see Apache 2.0 license.
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


%% Import gps data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Format:  1           2         3         4         5           6     7       8
%         start_time, end_time, gps_time, fix_mode, num_sat,    lat,  long,   elev,
%         9       10       11       12       13       14       15    16    17    18    19
%         ecef_x, ecef_y,  ecef_z,  ecef_vx, ecef_vy, ecef_vz, gdop, pdop, hdop, vdop, tdop
if ~exist('last_lowcost_dir', 'var') || ~ischar(last_lowcost_dir)
   last_lowcost_dir = last_applanix_dir; 
end
disp('Please select Skytraq log file')
[skytraq_file, skytraq_path] = uigetfile(text_files, 'Select Skytraq Log File', last_lowcost_dir);
last_lowcost_dir = skytraq_path;

gps = csvread([skytraq_path skytraq_file],1,0);
gps_time = gps(:,1);
% grab lat-long
lla = gps(:,6:8);
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
%% Correct time racnges %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timelag = -0.0963798;
[start_time,Is] = max([min(gps_time),min(gt_t),min(imu(:,1))]);
[end_time,Ie] = min([max(gps_time),max(gt_t),max(imu(:,1))]);
end_time = end_time - start_time;
gt_t = gt_t - start_time;
gps_time = gps_time - start_time + timelag;
imu(:,1) = imu(:,1) - start_time;
% cut off extra gps data at end, fold time back in
gps_mask = gps_time < end_time & gps_time > 0;
gps_time = gps_time(gps_mask);
gps = gps(gps_mask,:);
lla = lla(gps_mask,:);
gps(:,1) = gps_time;
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
gps_x = gps_x(gps_mask) - min_x;
gps_y = gps_y(gps_mask) - min_y;
gps_h = gps_h(gps_mask);
ground_truth = zeros(length(gps_time), 6);
ground_truth_full = zeros(length(filter_time), 9);

ground_truth(:,1) =  (interp1(gt_t,gt_xyz(:,2),gps_time)-min_x)';
ground_truth(:,2) =  (interp1(gt_t,gt_xyz(:,1),gps_time)-min_y)';
ground_truth(:,3) =  interp1(gt_t,gt_xyz(:,3),gps_time)';
ground_truth(:,4) =  interp1(gt_t,gt_rot(:,1),gps_time)';
ground_truth(:,5) =  interp1(gt_t,gt_rot(:,2),gps_time)';
ground_truth(:,6) =  interp1(gt_t,gt_rot(:,3),gps_time)';

ground_truth_full(:,1) = (interp1(gt_t,gt_xyz(:,2),filter_time)-min_x)';
ground_truth_full(:,2) = (interp1(gt_t,gt_xyz(:,1),filter_time)-min_y)';
ground_truth_full(:,3) = interp1(gt_t,gt_xyz(:,3),filter_time)';
ground_truth_full(:,4) = interp1(gt_t,gt_rot(:,1),filter_time)';
ground_truth_full(:,5) = interp1(gt_t,gt_rot(:,2),filter_time)';
ground_truth_full(:,6) = interp1(gt_t,gt_rot(:,3),filter_time)';
ground_truth_full(:,7) = interp1(gt_t,gt_vel(:,1),filter_time)';
ground_truth_full(:,8) = interp1(gt_t,gt_vel(:,2),filter_time)';
ground_truth_full(:,9) = interp1(gt_t,gt_vel(:,3),filter_time)';