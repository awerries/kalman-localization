%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.

% Specify range
epoch = linspace(0.01,0.1,200);
num_items = length(epoch);
rms_error_filter = Inf*ones(1,num_items);
max_error_filter = Inf*ones(1,num_items);
parfor i = 1:num_items
    fprintf('Iteration: %d, Epoch: %08.7f\n', i, epoch(i));
    temp_filter_time = 0:epoch(i):imu_time(length(imu_time));
    [out_profile,out_IMU_bias_est,out_KF_SD] = ...
        Loosely_coupled_INS_GNSS(init_cond, temp_filter_time, epoch(i), lla, gps, imu, LC_KF_config, est_IMU_bias);
    xyz = out_profile(:,2:4);
    if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
        llh = ecef2lla(xyz);
        [x,y] = deg2utm(llh(:,1),llh(:,2));
        x = x-min_x;
        y = y-min_y;
        ground_truth_full = zeros(length(temp_filter_time), 9);
        ground_truth_full(:,1) = (interp1(gt_t,gt_xyz(:,2),temp_filter_time)-min_x)';
        ground_truth_full(:,2) = (interp1(gt_t,gt_xyz(:,1),temp_filter_time)-min_y)';
        distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
        rms_error_filter(i) = rms(distance);
        max_error_filter(i) = max(distance);
    end
end

[minmax, i] = min(max_error_filter);
fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, epoch: %08.7f\n', i, epoch(i));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, epoch: %08.7f\n', i, epoch(i));
