%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.

k_max = 30;
% Specify ranges
pos_meas_SD = linspace(0.05,20,40);
vel_meas_SD = linspace(0.05,5,40);
% Repeat arrays
pos_meas_SD = repmat(pos_meas_SD, [1 k_max]);
vel_meas_SD = repmat(vel_meas_SD, [1 k_max]);
% Generate random selections of each vector
num_items = length(pos_meas_SD);
pos_i = randperm(num_items);
vel_i = randperm(num_items);
rms_error_filter = Inf*ones(1,num_items);
max_error_filter = Inf*ones(1,num_items);
parfor i = 1:num_items
    fprintf('Iteration: %d, PosSD: %08.5f, VelSD: %08.5f\n', i, pos_meas_SD(pos_i(i)), vel_meas_SD(vel_i(i)));
    temp_conf = LC_KF_config;
    temp_conf.pos_meas_SD = pos_meas_SD(pos_i(i));
    temp_conf.vel_meas_SD = vel_meas_SD(vel_i(i));
    [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, novatel, imu, temp_conf, est_IMU_bias);
    xyz = out_profile(:,2:4);
    if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
        llh = ecef2lla(xyz);
        llhsize = size(llh)
        fprintf('Size of llh: %d %d\n', llhsize(1), llhsize(2));
        [x,y] = deg2utm(llh(:,1),llh(:,2));
        x = x-min_x;
        y = y-min_y;
        
        distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
        rms_error_filter(i) = rms(distance);
        max_error_filter(i) = max(distance);
    end
end

[minmax, i] = min(max_error_filter);
fprintf('\nBest max: %08.5f, rms is %08.5f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, PosSD: %08.5f, VelSD: %08.5f\n', i, pos_meas_SD(pos_i(i)), vel_meas_SD(vel_i(i)));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.5f, max is %08.5f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, PosSD: %08.5f, VelSD: %08.5f\n', i, pos_meas_SD(pos_i(i)), vel_meas_SD(vel_i(i)));
