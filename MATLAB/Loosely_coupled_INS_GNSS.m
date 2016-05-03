function [out_profile,out_IMU_bias_est,out_KF_SD,out_R_matrix,out_Q_matrix,corrections] =...
    Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, LC_KF_config, est_IMU_bias)
%Loosely_coupled_INS_GNSS - Simulates inertial navigation using ECEF
% navigation equations and kinematic model, GNSS using a least-squares
% positioning algorithm, and loosely-coupled INS/GNSS integration. 
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
%
% Significant changes made to data-formatting and time-handling by Adam
% Werries, 2015/2016.
%
% Inputs:
%   LC_KF_config
%     .init_att_unc           Initial attitude uncertainty per axis (rad)
%     .init_vel_unc           Initial velocity uncertainty per axis (m/s)
%     .init_pos_unc           Initial position uncertainty per axis (m)
%     .init_b_a_unc           Initial accel. bias uncertainty (m/s^2)
%     .init_b_g_unc           Initial gyro. bias uncertainty (rad/s)
%     .gyro_noise_PSD         Gyro noise PSD (rad^2/s)
%     .accel_noise_PSD        Accelerometer noise PSD (m^2 s^-3)
%     .accel_bias_PSD         Accelerometer bias random walk PSD (m^2 s^-5)
%     .gyro_bias_PSD          Gyro bias random walk PSD (rad^2 s^-3)
%     .pos_meas_SD            Position measurement noise SD per axis (m)
%     .vel_meas_SD            Velocity measurement noise SD per axis (m/s)
%     .init_biases            IMU bias initialization
%
% Outputs:
%   out_profile        Navigation solution as a motion profile array
%   out_IMU_bias_est   Kalman filter IMU bias estimate array
%   out_clock          GNSS Receiver clock estimate array
%   out_KF_SD          Output Kalman filter state uncertainties
%
% Format of motion profiles:
%  Column 1: time (sec)
%  Column 2: latitude (rad)
%  Column 3: longitude (rad)
%  Column 4: height (m)
%  Column 5: north velocity (m/s)
%  Column 6: east velocity (m/s)
%  Column 7: down velocity (m/s)
%  Column 8: roll angle of body w.r.t NED (rad)
%  Column 9: pitch angle of body w.r.t NED (rad)
%  Column 10: yaw angle of body w.r.t NED (rad)
%
% Format of output IMU biases array:
%  Column 1: time (sec)
%  Column 2: estimated X accelerometer bias (m/s^2)
%  Column 3: estimated Y accelerometer bias (m/s^2)
%  Column 4: estimated Z accelerometer bias (m/s^2)
%  Column 5: estimated X gyro bias (rad/s)
%  Column 6: estimated Y gyro bias (rad/s)
%  Column 7: estimated Z gyro bias (rad/s)
%
% Format of KF state uncertainties array:
%  Column 1: time (sec)
%  Column 2: X attitude error uncertainty (rad)
%  Column 3: Y attitude error uncertainty (rad)
%  Column 4: Z attitude error uncertainty (rad)
%  Column 5: X velocity error uncertainty (m/s)
%  Column 6: Y velocity error uncertainty (m/s)
%  Column 7: Z velocity error uncertainty (m/s)
%  Column 8: X position error uncertainty (m)
%  Column 9: Y position error uncertainty (m)
%  Column 10: Z position error uncertainty (m)
%  Column 11: X accelerometer bias uncertainty (m/s^2)
%  Column 12: Y accelerometer bias uncertainty (m/s^2)
%  Column 13: Z accelerometer bias uncertainty (m/s^2)
%  Column 14: X gyro bias uncertainty (rad/s)
%  Column 15: Y gyro bias uncertainty (rad/s)
%  Column 16: Z gyro bias uncertainty (rad/s)

old_time = filter_time(1);

% Initialize INS from GPS
old_est_r_eb_e = init_cond(1:3)';
old_est_v_eb_e = init_cond(4:6)';
[old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n] = pv_ECEF_to_NED(old_est_r_eb_e,old_est_v_eb_e);
est_L_b = old_est_L_b;

% Initialize estimated attitude solution
% old_est_C_b_n = Initialize_NED_attitude(true_C_b_n,initialization_errors);
old_est_C_b_n = Euler_to_CTM(init_cond(7:9));
[~,~,old_est_C_b_e] = NED_to_ECEF(old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n);
old_est_r_eb_e = old_est_r_eb_e + old_est_C_b_e*LC_KF_config.lever_arm + old_est_C_b_e*LC_KF_config.gps_correction;

% Initialize output profile record and errors record
out_profile = zeros(length(filter_time),10);

% Generate output profile record
out_profile(1,1) = old_time;
out_profile(1,2:4) = old_est_r_eb_e;
out_profile(1,5:7) = old_est_v_eb_e';
out_profile(1,8:10) = CTM_to_Euler(old_est_C_b_n')';

% Initialize Kalman filter P matrix and IMU bias states
P_matrix = Initialize_LC_P_matrix(LC_KF_config);

% Generate IMU bias and clock output records
out_IMU_bias_est(1,1) = old_time;
out_IMU_bias_est(1,2:7) = est_IMU_bias';

% Generate KF uncertainty record
out_KF_SD(1,1) = old_time;
for n = 1:15
    out_KF_SD(1,n+1) = sqrt(P_matrix(n,n));
end % for i

% Initialize R (not really used but it makes me feel better
pos_variance = LC_KF_config.gps_pos_stddev.^2*gps(1,16).^2.*ones(1,3);
vel_variance = LC_KF_config.gps_vel_stddev.^2*gps(1,16).^2.*ones(1,3);
R_matrix(1:3,1:3) = diag(max(LC_KF_config.pos_sd_min, ...
                         min(LC_KF_config.pos_sd_max,...
                             pos_variance)));
R_matrix(1:3,4:6) = zeros(3);
R_matrix(4:6,1:3) = zeros(3);
R_matrix(4:6,4:6) = diag(max(LC_KF_config.vel_sd_min,...
                         min(LC_KF_config.vel_sd_max,...
                         vel_variance)));

out_R_matrix = zeros(size(gps,1), 6);
for n = 1:6
    out_R_matrix(1,n) = R_matrix(n,n);
end % for i
out_Q_matrix = zeros(size(gps,1), 15);
Q_matrix = zeros(15,15);
for n = 1:15
    out_Q_matrix(1,n) = Q_matrix(n,n);
end % for i
% Main loop
GNSS_epoch = 2;
last_GNSS_epoch = GNSS_epoch;
corrections = zeros(15,size(gps,1)-1);
last_imu_index = 1;
numIMU = size(imu, 1);
imu_index_windowsize = floor(20*epoch/mean(diff(imu(:,1))));
for i = 2:length(filter_time)
    time = filter_time(i);
    % find range of imu measurements to use
    endcap = min(numIMU, last_imu_index+1 + imu_index_windowsize);
    indices = find(imu(last_imu_index+1:endcap,1) < time);
    imu_range_end = last_imu_index + indices(end) - 1;
    % Apply IMU bias estimates
    meas_f_ib_b = mean(imu(last_imu_index+1:imu_range_end,2:4))' - est_IMU_bias(1:3);
    meas_omega_ib_b = mean(imu(last_imu_index+1:imu_range_end,5:7))' - est_IMU_bias(4:6);
    
    % Update estimated navigation solution
    [est_r_eb_e,est_v_eb_e,est_C_b_e] = Nav_equations_ECEF(epoch,...
        old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,meas_f_ib_b,...
        meas_omega_ib_b);
%     while GNSS_epoch < size(gps,1)+1 && ~gps(GNSS_epoch,3)
%         GNSS_epoch = GNSS_epoch + 1;
%     end
    % Determine whether to update GNSS simulation and run Kalman filter
    if GNSS_epoch <= size(gps,1) && time > gps(GNSS_epoch,1)
        tor_s = gps(GNSS_epoch,1) - gps(last_GNSS_epoch,1);  % KF time interval
        GNSS_r_eb_e = gps(GNSS_epoch,9:11)';
        GNSS_v_eb_e = gps(GNSS_epoch,12:14)';
        est_L_b = lla(GNSS_epoch,1);
        % Use the GPS-reported standard deviation values, but clamping
        % min/max values according to configuration
        pos_variance = LC_KF_config.gps_pos_stddev.^2*gps(GNSS_epoch,16).^2;
        vel_variance = LC_KF_config.gps_vel_stddev.^2*gps(GNSS_epoch,16).^2;
        R_matrix(1:3,1:3) = diag(max(LC_KF_config.pos_sd_min, ...
                                     min(LC_KF_config.pos_sd_max,...
                                         pos_variance.*ones(1,3)*tor_s)));
        R_matrix(4:6,4:6) = diag(max(LC_KF_config.vel_sd_min,...
                                     min(LC_KF_config.vel_sd_max,...
                                         vel_variance.*ones(1,3)*tor_s)));
%         disp(size(R_matrix));
%         disp(diag(R_matrix));
        
        % Run Integration Kalman filter
        [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix_new,corrections(:,GNSS_epoch-1), Phi_matrix, Q_matrix] =...
            LC_KF_Epoch(GNSS_epoch, GNSS_r_eb_e,GNSS_v_eb_e,tor_s,est_C_b_e,...
            est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,...
            est_L_b,LC_KF_config,Q_matrix,R_matrix,meas_omega_ib_b);
        if any(any(isnan(P_matrix))) || any(any(isinf(P_matrix)))
            disp('Filter instability detected. Time to kick the bucket.');
            fprintf('Died on iteration %d/%d\n',i,length(filter_time));
            fprintf('and GNSS epoch %d/%d\n',GNSS_epoch,size(gps,1));
            return
        end

        % Run adaptive algorithm
        [Q_matrix] = adapt_noise_covariance(Phi_matrix, P_matrix_new, P_matrix, Q_matrix, ...
                                            LC_KF_config.n, GNSS_epoch, corrections);
        P_matrix = P_matrix_new;
        
        % Generate IMU bias and clock output records
        out_IMU_bias_est(GNSS_epoch,1) = time;
        out_IMU_bias_est(GNSS_epoch,2:7) = est_IMU_bias';

        % Generate KF uncertainty output record
        out_KF_SD(GNSS_epoch,1) = time;
        for n = 1:15
            out_KF_SD(GNSS_epoch,n+1) = sqrt(P_matrix(n,n));
        end % for i
        for n = 1:6
            out_R_matrix(GNSS_epoch,n) = sqrt(abs(R_matrix(n,n)));
        end % for i
        for n = 1:15
            out_Q_matrix(GNSS_epoch,n) = sqrt(abs(Q_matrix(n,n)));
        end % for i
        last_GNSS_epoch = GNSS_epoch;
        GNSS_epoch = GNSS_epoch + 1;
    end % if time    
    
    % Convert navigation solution to NED
    [~,~,~,~,est_C_b_n] =...
        ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);

    % Generate output profile record
    out_profile(i,1) = time;
    out_profile(i,2:4) = est_r_eb_e;
    out_profile(i,5:7) = est_v_eb_e';
    out_profile(i,8:10) = CTM_to_Euler(est_C_b_n')';
    
    % Reset old values
    old_est_r_eb_e = est_r_eb_e;
    old_est_v_eb_e = est_v_eb_e;
    old_est_C_b_e = est_C_b_e;
    last_imu_index = imu_range_end;
end %epoch

% Ends