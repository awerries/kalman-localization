%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% pos_sd_min = linspace(0,2,2000);
% num_items = length(pos_sd_min);
% rms_error_filter = Inf*ones(1,num_items);
% max_error_filter = Inf*ones(1,num_items);
% parfor i = 1:num_items
%     fprintf('Iteration: %d, pos_sd_min: %08.7f\n', i, pos_sd_min(i));
%     temp_conf = LC_KF_config;
%     temp_conf.pos_sd_min = pos_sd_min(i);
%     [out_profile,out_IMU_bias_est,out_KF_SD, out_R_matrix, out_Q_matrix, corrections] = ...
%         Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
%     xyz = out_profile(:,2:4);
%     if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
%         llh = ecef2lla(xyz);
%         [x,y] = deg2utm(llh(:,1),llh(:,2));
%         x = x-min_x;
%         y = y-min_y;
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
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
% figurec;
% subplot(211);
% plot(pos_sd_min, rms_error_filter);
% xlabel('pos_sd_min');
% ylabel('max error');
% 
% subplot(212);
% plot(pos_sd_min, max_error_filter);
% xlabel('pos_sd_min');
% ylabel('max error');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pos_sd_max = linspace(0,200,2000);
% num_items = length(pos_sd_max);
% rms_error_filter = Inf*ones(1,num_items);
% max_error_filter = Inf*ones(1,num_items);
% parfor i = 1:num_items
%     fprintf('Iteration: %d, pos_sd_max: %08.7f\n', i, pos_sd_max(i));
%     temp_conf = LC_KF_config;
%     temp_conf.pos_sd_max = pos_sd_max(i);
%     [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
%     xyz = out_profile(:,2:4);
%     if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
%         llh = ecef2lla(xyz);
%         [x,y] = deg2utm(llh(:,1),llh(:,2));
%         x = x-min_x;
%         y = y-min_y;
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
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
% figurec;
% plot(pos_sd_max, max_error_filter); hold on;
% plot(pos_sd_max, rms_error_filter);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vel_sd_min = linspace(0,30,2000);
% num_items = length(vel_sd_min);
% rms_error_filter = Inf*ones(1,num_items);
% max_error_filter = Inf*ones(1,num_items);
% parfor i = 1:num_items
%     fprintf('Iteration: %d, vel_sd_min: %08.7f\n', i, vel_sd_min(i));
%     temp_conf = LC_KF_config;
%     temp_conf.vel_sd_min = vel_sd_min(i);
%     [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
%     xyz = out_profile(:,2:4);
%     if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
%         llh = ecef2lla(xyz);
%         [x,y] = deg2utm(llh(:,1),llh(:,2));
%         x = x-min_x;
%         y = y-min_y;
% %         h = -llh(:,3);
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
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
% figure;
% plot(vel_sd_min, max_error_filter); hold on;
% plot(vel_sd_min, rms_error_filter);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vel_sd_max = linspace(1,50,200);
num_items = length(vel_sd_max);
rms_error_filter = Inf*ones(1,num_items);
max_error_filter = Inf*ones(1,num_items);
parfor i = 1:num_items
    fprintf('Iteration: %d, vel_sd_max: %08.7f\n', i, vel_sd_max(i));
    temp_conf = LC_KF_config;
    temp_conf.vel_sd_max = vel_sd_max(i);
    [out_profile,out_IMU_bias_est,out_KF_SD, out_R_matrix, out_Q_matrix, corrections] = ...
        Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
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
fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, vel_sd_max: %08.7f\n', i, vel_sd_max(i));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, vel_sd_max: %08.7f\n', i, vel_sd_max(i));
figurec;
plot(vel_sd_max, max_error_filter); hold on;
plot(vel_sd_max, rms_error_filter);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% gps_pos_stddev = linspace(.001,10,2000);
% num_items = length(gps_pos_stddev);
% rms_error_filter = Inf*ones(1,num_items);
% max_error_filter = Inf*ones(1,num_items);
% parfor i = 1:num_items
%     fprintf('Iteration: %d, gps_pos_stddev: %08.7f\n', i, gps_pos_stddev(i));
%     temp_conf = LC_KF_config;
%     temp_conf.gps_pos_stddev = gps_pos_stddev(i);
%     [out_profile,out_IMU_bias_est,out_KF_SD, out_R_matrix, out_Q_matrix, corrections] = ...
%         Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
%     xyz = out_profile(:,2:4);
%     if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
%         llh = ecef2lla(xyz);
%         [x,y] = deg2utm(llh(:,1),llh(:,2));
%         x = x-min_x;
%         y = y-min_y;
% %         h = -llh(:,3);
% %         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
%         rms_error_filter(i) = rms(distance);
%         max_error_filter(i) = max(distance);
%     end
% end
% 
% [minmax, i] = min(max_error_filter);
% fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
% fprintf('Best iteration for max: %d, gps_pos_stddev: %08.7f\n', i, gps_pos_stddev(i));
% [minrms, i] = min(rms_error_filter);
% fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
% fprintf('Best iteration for rms: %d, gps_pos_stddev: %08.7f\n', i, gps_pos_stddev(i));
% figurec;
% plot(gps_pos_stddev, max_error_filter); hold on;
% plot(gps_pos_stddev, rms_error_filter);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% gps_vel_stddev = linspace(.01,100,2000);
% num_items = length(gps_vel_stddev);
% rms_error_filter = Inf*ones(1,num_items);
% max_error_filter = Inf*ones(1,num_items);
% parfor i = 1:num_items
%     fprintf('Iteration: %d, vel_sd_max: %08.7f\n', i, gps_vel_stddev(i));
%     temp_conf = LC_KF_config;
%     temp_conf.gps_vel_stddev = gps_vel_stddev(i);
%     [out_profile,out_IMU_bias_est,out_KF_SD, out_R_matrix, out_Q_matrix, corrections] = ...
%         Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
%     xyz = out_profile(:,2:4);
%     if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
%         llh = ecef2lla(xyz);
%         [x,y] = deg2utm(llh(:,1),llh(:,2));
%         x = x-min_x;
%         y = y-min_y;
% %         h = -llh(:,3);
% %         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;        rms_error_filter(i) = rms(distance);
%         max_error_filter(i) = max(distance);
%     end
% end
% 
% [minmax, i] = min(max_error_filter);
% fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
% fprintf('Best iteration for max: %d, gps_vel_stddev: %08.7f\n', i, gps_vel_stddev(i));
% [minrms, i] = min(rms_error_filter);
% fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
% fprintf('Best iteration for rms: %d, gps_vel_stddev: %08.7f\n', i, gps_vel_stddev(i));
% figurec;
% plot(gps_vel_stddev, max_error_filter); hold on;
% plot(gps_vel_stddev, rms_error_filter);

load handel
sound(y,Fs)