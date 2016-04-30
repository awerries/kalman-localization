%% brute_force_tune
% Code to tune parameters by brute force. This is for heading init.
% Obviously theoretically completely silly.
%
% Works sorta like RANSAC I guess?
%
% Adam Werries 2016, see Apache 2.0 license.

k_max = 60;
% Specify ranges
roll = linspace(-45,45,100);
pitch = linspace(-45,45,100);
yaw = linspace(-180,180,100);
% Repeat arrays
roll = repmat(roll, [1 k_max]);
pitch = repmat(pitch, [1 k_max]);
yaw = repmat(yaw, [1 k_max]);
% Generate random selections of each vector
num_items = length(roll);
roll_i = randperm(num_items);
pitch_i = randperm(num_items);
yaw_i = randperm(num_items);
rms_error_filter = zeros(1,num_items);
max_error_filter = zeros(1,num_items);
parfor i = 1:num_items
    fprintf('Iteration: %d, Roll: %08.5f, Pitch: %08.5f, Yaw: %08.5f\n', i, roll(roll_i(i)), pitch(pitch_i(i)), yaw(yaw_i(i)));
    temp_cond = init_cond;
    temp_cond(7:9) = [deg2rad(roll(roll_i(i))) deg2rad(pitch(pitch_i(i))) deg2rad(yaw(yaw_i(i)))];
    [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(temp_cond, filter_time, epoch, lla, novatel, imu, LC_KF_config, est_IMU_bias);
    xyz = out_profile(:,2:4);
    llh = ecef2lla(xyz);
    [x,y] = deg2utm(llh(:,1),llh(:,2));
    x = x-min_x;
    y = y-min_y;
    
    distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
    rms_error_filter(i) = rms(distance);
    max_error_filter(i) = max(distance);
end

[minmax, i] = min(max_error_filter);
fprintf('\nBest max: %08.5f, rms is %08.5f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, Roll: %08.5f, Pitch: %08.5f, Yaw: %08.5f\n', i, roll(roll_i(i)), pitch(pitch_i(i)), yaw(yaw_i(i)));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.5f, max is %08.5f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, Roll: %08.5f, Pitch: %08.5f, Yaw: %08.5f\n', i, roll(roll_i(i)), pitch(pitch_i(i)), yaw(yaw_i(i)));
[minrms, i] = min((rms_error_filter+max_error_filter)/2);
fprintf('Best average of RMS and max: %08.4f, rms is  %08.4f, max is %08.4f\n', minrms, rms_error_filter(i), max_error_filter(i));
fprintf('Best iteration for rms: %d, Roll: %08.5f, Pitch: %08.5f, Yaw: %08.5f\n', i, roll(roll_i(i)), pitch(pitch_i(i)), yaw(yaw_i(i)));