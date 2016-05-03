%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.

k_max = 50;
% Specify ranges
init_att_unc = linspace(0.01,10,100);
init_vel_unc = linspace(0.01,10,100);
init_pos_unc = linspace(0.1,40,100);
% Repeat arrays
init_att_unc = repmat(init_att_unc, [1 k_max]);
init_vel_unc = repmat(init_vel_unc, [1 k_max]);
init_pos_unc = repmat(init_pos_unc, [1 k_max]);
% Generate random selections of each vector
num_items = length(init_att_unc);
att_i = randperm(num_items);
vel_i = randperm(num_items);
pos_i = randperm(num_items);
rms_error_filter = zeros(1,num_items);
max_error_filter = zeros(1,num_items);
parfor i = 1:num_items
    fprintf('Iteration: %d, AttUnc: %08.5f, VelUnc: %08.5f, PosUnc: %08.5f\n', i, init_att_unc(att_i(i)), init_vel_unc(vel_i(i)), init_pos_unc(pos_i(i)));
    temp_conf = LC_KF_config;
    temp_conf.init_att_unc = deg2rad(init_att_unc(att_i(i)));
    temp_conf.init_vel_unc = init_vel_unc(vel_i(i));
    temp_conf.init_pos_unc = init_pos_unc(pos_i(i));
    [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
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
fprintf('Best iteration for max: %d, AttUnc: %08.5f, VelUnc: %08.5f, PosUnc: %08.5f\n', i, init_att_unc(att_i(i)), init_vel_unc(vel_i(i)), init_pos_unc(pos_i(i)));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.5f, max is %08.5f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, AttUnc: %08.5f, VelUnc: %08.5f, PosUnc: %08.5f\n', i, init_att_unc(att_i(i)), init_vel_unc(vel_i(i)), init_pos_unc(pos_i(i)));
[minrms, i] = min((rms_error_filter+max_error_filter)/2);
fprintf('Best average of RMS and max: %08.5f, rms is  %08.5f, max is %08.5f\n', minrms, rms_error_filter(i), max_error_filter(i));
fprintf('Best iteration for average: %d, AttUnc: %08.5f, VelUnc: %08.5f, PosUnc: %08.5f\n', i, init_att_unc(att_i(i)), init_vel_unc(vel_i(i)), init_pos_unc(pos_i(i)));