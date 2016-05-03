%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.

k_max = 50;
% Specify ranges
accel_bias_PSD = logspace(-10,-4,100);
gyro_bias_PSD = logspace(-10,-4,100);
% Repeat arrays
accel_bias_PSD = repmat(accel_bias_PSD, [1 k_max]);
gyro_bias_PSD = repmat(gyro_bias_PSD, [1 k_max]);
% Generate random selections of each vector
num_items = length(accel_bias_PSD);
accel_bias_i = randperm(num_items);
gyro_bias_i = randperm(num_items);

rms_error_filter = Inf*ones(1,num_items);
max_error_filter = Inf*ones(1,num_items);
parfor i = 1:num_items
    fprintf('Iteration: %d, ABias: %08.5e, GBias: %08.5e\n', i, accel_bias_PSD(accel_bias_i(i)), gyro_bias_PSD(gyro_bias_i(i)));
    temp_conf = LC_KF_config;
    temp_conf.accel_bias_PSD = accel_bias_PSD(accel_bias_i(i));
    temp_conf.gyro_bias_PSD = gyro_bias_PSD(gyro_bias_i(i));
    [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
    xyz = out_profile(:,2:4);
    if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
        llh = ecef2lla(xyz);
        [x,y] = deg2utm(llh(:,1),llh(:,2));
        x = x-min_x;
        y = y-min_y;
%         h = -llh(:,3);
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
        distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
        rms_error_filter(i) = rms(distance);
        max_error_filter(i) = max(distance);
    end
end

[minmax, i] = min(max_error_filter);
fprintf('\nBest max: %08.4f, rms is %08.4f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, ABias: %08.5e, GBias: %08.5e\n', i, accel_bias_PSD(accel_bias_i(i)), gyro_bias_PSD(gyro_bias_i(i)));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.4f, max is %08.4f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, ABias: %08.5e, GBias: %08.5e\n', i, accel_bias_PSD(accel_bias_i(i)), gyro_bias_PSD(gyro_bias_i(i)));
[minrms, i] = min((rms_error_filter+max_error_filter)/2);
fprintf('Best average of RMS and max: %08.4f, rms is  %08.4f, max is %08.4f\n', minrms, rms_error_filter(i), max_error_filter(i));
fprintf('Best iteration for rms: %d, ABias: %08.5e, GBias: %08.5e\n', i, accel_bias_PSD(accel_bias_i(i)), gyro_bias_PSD(gyro_bias_i(i)));
