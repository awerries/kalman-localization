%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pos_sd_min = linspace(1,4,200);
% num_items = length(pos_sd_min);
% rms_error_filter = Inf*ones(1,num_items);
% max_error_filter = Inf*ones(1,num_items);
% parfor i = 1:num_items
%     fprintf('Iteration: %d, pos_sd_min: %08.7f\n', i, pos_sd_min(i));
%     temp_conf = LC_KF_config;
%     temp_conf.pos_sd_min = pos_sd_min(i);
%     [out_profile,out_IMU_bias_est,out_KF_SD, out_R_matrix, out_Q_matrix, corrections] = ...
%         Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, novatel, imu, temp_conf, est_IMU_bias);
%     xyz = out_profile(:,2:4);
%     if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
%         llh = ecef2lla(xyz);
%         [x,y] = deg2utm(llh(:,1),llh(:,2));
%         x = x-min_x;
%         y = y-min_y;
%         h = -llh(:,3);
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
%         rms_error_filter(i) = rms(distance);
%         max_error_filter(i) = max(distance);
%     end
% end
% 
% [minmax, i] = min(max_error_filter);
% fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
% fprintf('Best iteration for max: %d, pos_sd_min: %08.7f\n', i, pos_sd_min(i));
% [minrms, i] = min(rms_error_filter);
% fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
% fprintf('Best iteration for rms: %d, pos_sd_min: %08.7f\n', i, pos_sd_min(i));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pos_sd_max = linspace(40,100,300);
% num_items = length(pos_sd_max);
% rms_error_filter = Inf*ones(1,num_items);
% max_error_filter = Inf*ones(1,num_items);
% parfor i = 1:num_items
%     fprintf('Iteration: %d, pos_sd_max: %08.7f\n', i, pos_sd_max(i));
%     temp_conf = LC_KF_config;
%     temp_conf.pos_sd_max = pos_sd_max(i);
%     [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, novatel, imu, temp_conf, est_IMU_bias);
%     xyz = out_profile(:,2:4);
%     if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
%         llh = ecef2lla(xyz);
%         [x,y] = deg2utm(llh(:,1),llh(:,2));
%         x = x-min_x;
%         y = y-min_y;
%         h = -llh(:,3);
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
%         rms_error_filter(i) = rms(distance);
%         max_error_filter(i) = max(distance);
%     end
% end
% 
% [minmax, i] = min(max_error_filter);
% fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
% fprintf('Best iteration for max: %d, pos_sd_max: %08.7f\n', i, pos_sd_max(i));
% [minrms, i] = min(rms_error_filter);
% fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
% fprintf('Best iteration for rms: %d, pos_sd_max: %08.7f\n', i, pos_sd_max(i));

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vel_sd_min = linspace(0,3,500);
% num_items = length(vel_sd_min);
% rms_error_filter = Inf*ones(1,num_items);
% max_error_filter = Inf*ones(1,num_items);
% parfor i = 1:num_items
%     fprintf('Iteration: %d, vel_sd_min: %08.7f\n', i, vel_sd_min(i));
%     temp_conf = LC_KF_config;
%     temp_conf.vel_sd_min = vel_sd_min(i);
%     [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, novatel, imu, temp_conf, est_IMU_bias);
%     xyz = out_profile(:,2:4);
%     if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
%         llh = ecef2lla(xyz);
%         [x,y] = deg2utm(llh(:,1),llh(:,2));
%         x = x-min_x;
%         y = y-min_y;
%         h = -llh(:,3);
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
%         rms_error_filter(i) = rms(distance);
%         max_error_filter(i) = max(distance);
%     end
% end
% 
% [minmax, i] = min(max_error_filter);
% fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
% fprintf('Best iteration for max: %d, vel_sd_min: %08.7f\n', i, vel_sd_min(i));
% [minrms, i] = min(rms_error_filter);
% fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
% fprintf('Best iteration for rms: %d, vel_sd_min: %08.7f\n', i, vel_sd_min(i));
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vel_sd_max = linspace(3,20,500);
num_items = length(vel_sd_max);
rms_error_filter = Inf*ones(1,num_items);
max_error_filter = Inf*ones(1,num_items);
parfor i = 1:num_items
    fprintf('Iteration: %d, vel_sd_max: %08.7f\n', i, vel_sd_max(i));
    temp_conf = LC_KF_config;
    temp_conf.vel_sd_max = vel_sd_max(i);
    [out_profile,out_IMU_bias_est,out_KF_SD, out_R_matrix, out_Q_matrix, corrections] = ...
        Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, novatel, imu, temp_conf, est_IMU_bias);
    xyz = out_profile(:,2:4);
    if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
        llh = ecef2lla(xyz);
        [x,y] = deg2utm(llh(:,1),llh(:,2));
        x = x-min_x;
        y = y-min_y;
        h = -llh(:,3);
        distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
        rms_error_filter(i) = rms(distance);
        max_error_filter(i) = max(distance);
    end
end

[minmax, i] = min(max_error_filter);
fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, vel_sd_max: %08.7f\n', i, vel_sd_max(i));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, vel_sd_max: %08.7f\n', i, vel_sd_max(i));