% Code to test the performance of various tuning parameters
% Adam Werries 2016, see Apache 2.0 license.

parameter = 100:20:400;
rms_error_filter = zeros(1,length(parameter));
max_error_filter = zeros(1,length(parameter));
parfor i = 1:length(parameter)
    fprintf('Testing parameter: %g\n', parameter(i));
    n = parameter(i);
    [out_profile,out_IMU_bias_est,out_KF_SD] = Loosely_coupled_INS_GNSS(init_cond, lla, novatel, imu, LC_KF_config,n);
    xyz = out_profile(:,2:4);
    llh = ecef2lla(xyz);
    [x,y] = deg2utm(llh(:,1),llh(:,2));
    x = x-min_x;
    y = y-min_y;

    distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
    rms_error_filter(i) = rms(distance);
    max_error_filter(i) = max(distance);
end

[minmax, i] = min(max_error_filter)
fprintf('\nBest parameter for max: %g\n', parameter(i));
[minrms, i] = min(rms_error_filter)
fprintf('\nBest parameter for rms: %g\n', parameter(i));
