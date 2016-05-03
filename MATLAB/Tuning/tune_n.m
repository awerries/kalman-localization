%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.

% Specify ranges
n = 1:1:100;
num_items = length(n);
rms_error_filter = Inf*ones(1,num_items);
max_error_filter = Inf*ones(1,num_items);
parfor i = 1:num_items
    temp_conf = LC_KF_config;
    temp_conf.n = n(i);
    [out_profile,~,~,~,~] = Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, temp_conf, est_IMU_bias);
    xyz = out_profile(:,2:4);
    if ~any(any(isnan(xyz))) && ~any(any(isinf(xyz)))
        llh = ecef2lla(xyz);
        llhsize = size(llh)
        [x,y] = deg2utm(llh(:,1),llh(:,2));
        x = x-min_x;
        y = y-min_y;
        h = -llh(:,3);
%         distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
        distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
        rms_error_filter(i) = rms(distance);
        max_error_filter(i) = max(distance);
        fprintf('Iteration: %d, n: %d, rms: %08.5f, max:  %08.5f\n', i, n(i), rms_error_filter(i),max_error_filter(i));
    end
end

[minmax, i] = min(max_error_filter);
fprintf('\nBest max: %08.5f, rms is %08.5f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, n: %d\n', i, n(i));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.5f, max is %08.5f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, n: %d\n', i, n(i));
[minrms, i] = min((rms_error_filter+max_error_filter)/2);
fprintf('Best average of RMS and max: %08.4f, rms is  %08.4f, max is %08.4f\n', minrms, rms_error_filter(i), max_error_filter(i));
fprintf('Best iteration for rms: %d, n: %d\n', i, n(i));

figurec;
plot(n, max_error_filter); hold on;
plot(n, rms_error_filter);


load handel
sound(y,Fs)