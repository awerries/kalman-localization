%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.

k_max = 40;
% Specify ranges
init_b_a_unc = linspace(10,3000,100);
init_b_g_unc = linspace(0,500,100);
% Repeat arrays
init_b_a_unc = repmat(init_b_a_unc, [1 k_max]);
init_b_g_unc = repmat(init_b_g_unc, [1 k_max]);
% Generate random selections of each vector
num_items = length(init_b_a_unc);
ba_i = randperm(num_items);
bg_i = randperm(num_items);
rms_error_filter = zeros(1,num_items);
max_error_filter = zeros(1,num_items);
parfor i = 1:num_items
    fprintf('Iteration: %d, aBiasUnc: %08.3f, gBiasUnc: %08.3f\n', i, init_b_a_unc(ba_i(i)), init_b_g_unc(bg_i(i)));
    temp_conf = LC_KF_config;
    temp_conf.init_b_a_unc = init_b_a_unc(ba_i(i)) * mug_to_mps2;
    temp_conf.init_b_g_unc = init_b_g_unc(bg_i(i)) * deg_to_rad / 3600;
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
fprintf('\nBest max: %08.3f, rms is %08.3f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, aBiasUnc: %08.3f, gBiasUnc: %08.3f\n', i, init_b_a_unc(ba_i(i)), init_b_g_unc(bg_i(i)));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.3f, max is %08.3f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, aBiasUnc: %08.3f, gBiasUnc: %08.3f\n', i, init_b_a_unc(ba_i(i)), init_b_g_unc(bg_i(i)));
