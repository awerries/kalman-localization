%% brute_force_tune
% Code to test the performance of various tuning parameters
% Works sorta like RANSAC I guess?
% Adam Werries 2016, see Apache 2.0 license.

% Specify range
timelag = linspace(-0.25,0.25,10000);
num_items = length(timelag);
rms_error_filter = Inf*ones(1,num_items);
max_error_filter = Inf*ones(1,num_items);
for i = 1:num_items
    ground_truth_temp = zeros(length(filter_time), 9);
    ground_truth_temp(:,1) = (interp1(gt_t+timelag(i),gt_xyz(:,2),filter_time)-min_x)';
    ground_truth_temp(:,2) = (interp1(gt_t+timelag(i),gt_xyz(:,1),filter_time)-min_y)';
    distance = ((ground_truth_temp(:,1)-x).^2 + (ground_truth_temp(:,2)-y).^2).^0.5;
    rms_error_filter(i) = rms(distance);
    max_error_filter(i) = max(distance);
end

[minmax, i] = min(max_error_filter);
fprintf('\nBest max: %08.7f, rms is %08.7f\n', minmax, rms_error_filter(i));
fprintf('Best iteration for max: %d, timelag: %08.7f\n', i, timelag(i));
[minrms, i] = min(rms_error_filter);
fprintf('Best rms: %08.7f, max is %08.7f\n', minrms, max_error_filter(i));
fprintf('Best iteration for rms: %d, timelag: %08.7f\n', i, timelag(i));
[minrms, i] = min((rms_error_filter+max_error_filter)/2);
fprintf('Best average of RMS and max: %08.4f, rms is  %08.4f, max is %08.4f\n', minrms, rms_error_filter(i), max_error_filter(i));
fprintf('Best iteration for rms: %d, timelag: %08.7f\n', i, timelag(i));