%% Run the filters and plot 
% Adam Werries 2016, see Apache 2.0 license.
initialize;

%% Initialize Error Estimates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constants
deg_to_rad = 0.01745329252;
rad_to_deg = 1/deg_to_rad;
mug_to_mps2 = 9.80665E-6;

% CONFIGURATION
% Initial attitude uncertainty per axis (deg, converted to rad)
LC_KF_config.init_att_unc = degtorad(1.5);
% Initial velocity uncertainty per axis (m/s)
LC_KF_config.init_vel_unc = 1.62;
% Initial position uncertainty per axis (m)
LC_KF_config.init_pos_unc = 0.1;
% Initial accelerometer bias uncertainty (micro-g, converted
% to m/s^2)
LC_KF_config.init_b_a_unc = 40 * mug_to_mps2;
% Initial gyro bias uncertainty(deg/hour, converted to rad/sec)
LC_KF_config.init_b_g_unc = 353 * deg_to_rad / 3600;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
LC_KF_config.gyro_noise_PSD = (30 * deg_to_rad / 60)^2;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
LC_KF_config.accel_noise_PSD = (1000 * mug_to_mps2)^2;
% Accelerometer bias random walk PSD (m^2 s^-5)
LC_KF_config.accel_bias_PSD = 1e-10;
% Gyro bias random walk PSD (rad^2 s^-3)
LC_KF_config.gyro_bias_PSD = 1e-10;
% Lever arm from IMU to GPS
LC_KF_config.lever_arm = [0; 0.3046; 0.9209209];
LC_KF_config.gps_correction = [0; -0.1783567; -1.5190381];
% Minimum and maximum R matrix values
LC_KF_config.gps_pos_stddev = 1.52;
LC_KF_config.gps_vel_stddev = 16;
LC_KF_config.pos_sd_min = 0;
LC_KF_config.pos_sd_max = 100;
LC_KF_config.vel_sd_min = 5.1;
LC_KF_config.vel_sd_max = 100;
% Initial estimate of accelerometer and gyro static bias
est_IMU_bias = [
   0.079275626459682
   0.241364047070897
   0.734772061774872
  -0.029153756601168
  -0.000357971882895
  -0.015669929574187];
% number of measurements to use for innovation adaptive estimation
% LC_KF_config.n = 470;
LC_KF_config.n = Inf;

%% Format initial conditions
% x y z vx vy vz r p y
% init_cond = [gps(1,4:6) gps(1,10:12) deg2rad(-40) deg2rad(-8.5) deg2rad(-125)];
init_cond = [gps(1,9:11) gps(1,12:14) deg2rad(-13.1) deg2rad(-.455) deg2rad(-140)];

%% Loosely coupled ECEF INS and GNSS integrated navigation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Beginning processing');
% disp(LC_KF_config);
[out_profile,out_IMU_bias_est,out_KF_SD, out_R_matrix, out_Q_matrix, corrections] = ...
    Loosely_coupled_INS_GNSS(init_cond, filter_time, epoch, lla, gps, imu, LC_KF_config, est_IMU_bias);

generate_error_metrics
close all
generate_plots
mean(out_IMU_bias_est(:,2:end))'
