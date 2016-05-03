%% Raw IMU data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figurec;
subplot(311);
plot(imu_time, imu(:,2), '.');
xlabel('Time (s)'); ylabel('X Acceleration (m/s^2)');
subplot(312);
plot(imu_time, imu(:,3), '.');
xlabel('Time (s)'); ylabel('Y Acceleration (m/s^2)');
subplot(313);
plot(imu_time, imu(:,4), '.');
xlabel('Time (s)'); ylabel('Z Acceleration (m/s^2)');

figurec;
subplot(311);
plot(imu_time, imu(:,5), '.');
xlabel('Time (s)'); ylabel('Roll rate (rad/s)');
subplot(312);
plot(imu_time, imu(:,6), '.');
xlabel('Time (s)'); ylabel('Pitch rate (rad/s)');
subplot(313);
plot(imu_time, imu(:,7), '.');
xlabel('Time (s)'); ylabel('Yaw rate (rad/s)');

%% Compare ground truth to raw Skytraq data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figurec;
plot(gps_x, gps_y,'r--', 'LineWidth', 2);
xlabel('Easting (m)');
ylabel('Northing (m)');
hold on;
plot(ground_truth(:,1),ground_truth(:,2),'c');
legend('Skytraq','Applanix');
axis equal;

figurec;
subplot(311);
plot(gps_time, gps_x,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting (m)');
hold on; plot(gps_time, ground_truth(:,1),'c');
legend('Skytraq','Applanix');

subplot(312);
plot(gps_time, gps_y,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing (m)');
hold on; plot(gps_time, ground_truth(:,2),'c');
legend('Skytraq','Applanix','Location','SouthEast');

subplot(313);
plot(gps_time, gps_h,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Altitude (m)');
hold on; plot(gps_time, ground_truth(:,3),'c');
legend('Skytraq','Applanix','Location','SouthEast');

%% Plot filter results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
roll = out_profile(:,8);
pitch = out_profile(:,9);
yaw = out_profile(:,10);
figurec;
subplot(311);
plot(filter_time, rad2deg(roll),'-b'); hold on;
xlabel('Time (s)');
ylabel('Roll (deg)');

subplot(312);
plot(filter_time, rad2deg(pitch)); hold on;
xlabel('Time (s)');
ylabel('Pitch (deg)');
subplot(313);
plot(filter_time, rad2deg(yaw)); hold on;
xlabel('Time (s)');
ylabel('Yaw (deg)');

figurec;
plot(x, y,'r--', 'LineWidth', 2);
xlabel('Easting (m)');
ylabel('Northing (m)');
hold on;
plot(ground_truth(:,1),ground_truth(:,2),'c');
plot(gps_x, gps_y,'b-.', 'LineWidth', 2);
legend('Filtered','Applanix','Skytraq');
axis equal;

figurec;
subplot(311);
plot(filter_time, x,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting (m)');
hold on; plot(filter_time, ground_truth_full(:,1),'c');
legend('Filtered','Applanix');

subplot(312);
plot(filter_time, y,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing (m)');
hold on; plot(filter_time, ground_truth_full(:,2),'c');
legend('Filtered','Applanix','Location','SouthEast');

subplot(313);
plot(filter_time, h,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Altitude (m)');
hold on; plot(filter_time, ground_truth_full(:,3),'c');
legend('Filtered','Applanix','Location','SouthEast');

zero_line = zeros(length(filter_time), 1);
figurec;
subplot(311);
plot(filter_time, x-ground_truth_full(:,1),'r-', 'LineWidth', 2); hold on;
plot(gps_time, gps_x-ground_truth(:,1), '.g');
xlabel('Time (s)');
ylabel('Easting error (m)');
ylim([-10 10]);
legend('Filter','Skytraq');
plot(filter_time, zero_line, '--b');

subplot(312);
plot(filter_time, y-ground_truth_full(:,2),'r-', 'LineWidth', 2); hold on;
plot(gps_time, gps_y-ground_truth(:,2), '.g');
xlabel('Time (s)');
ylabel('Northing error (m)');
ylim([-10 10]);
legend('Filter','Skytraq');
plot(filter_time, zero_line, '--b');

subplot(313);
plot(filter_time, h-ground_truth_full(:,3),'r-', 'LineWidth', 2); hold on;
plot(gps_time, gps_h-ground_truth(:,3), '.g');
xlabel('Time (s)');
ylabel('Altitude error (m)');
ylim([-10 10]);
legend('Filter','Skytraq');
plot(filter_time, zero_line, '--b');

figurec;
subplot(311);

plot(gps_time, rad2deg(out_KF_SD(:,2)),'b'); hold on;
plot(gps_time, rad2deg(out_KF_SD(:,3)),'r');
plot(gps_time, rad2deg(out_KF_SD(:,4)),'g');
xlabel('Time (s)');
ylabel('P Att. Unc. (deg)');
legend('roll','pitch','yaw');

subplot(312);
plot(gps_time, out_KF_SD(:,5),'b'); hold on;
plot(gps_time, out_KF_SD(:,6),'r');
plot(gps_time, out_KF_SD(:,7),'g');
xlabel('Time (s)');
ylabel('P Vel. Unc. (m/s)');
legend('x', 'y', 'z');

subplot(313);
plot(gps_time, out_KF_SD(:,8),'b'); hold on;
plot(gps_time, out_KF_SD(:,9),'r');
plot(gps_time, out_KF_SD(:,10),'g');
xlabel('Time (s)');
ylabel('P Pos. Unc. (m)');
legend('x', 'y', 'z');

figurec;
subplot(211);
plot(gps_time, out_KF_SD(:,11),'b'); hold on;
plot(gps_time, out_KF_SD(:,12),'r');
plot(gps_time, out_KF_SD(:,13),'g');
xlabel('Time (s)');
ylabel('P Accel Bias Unc. (m/s^2)');
legend('x', 'y', 'z');

subplot(212);
plot(gps_time, out_KF_SD(:,14),'b'); hold on;
plot(gps_time, out_KF_SD(:,15),'r');
plot(gps_time, out_KF_SD(:,16),'g');
xlabel('Time (s)');
ylabel('P Gyro Bias Unc. (rad/s)');
legend('x', 'y', 'z');

figurec;
subplot(211);
plot(gps_time, out_R_matrix(:,1), 'b'); hold on;
plot(gps_time, out_R_matrix(:,2), 'r'); 
plot(gps_time, out_R_matrix(:,3), 'g'); 
ylabel('R Position Noise (m)');
xlabel('Time (s)');
legend('x','y','z');

subplot(212);
plot(gps_time, out_R_matrix(:,4), 'b'); hold on;
plot(gps_time, out_R_matrix(:,5), 'r'); 
plot(gps_time, out_R_matrix(:,6), 'g'); 
ylabel('R Velocity Noise (m)');
xlabel('Time (s)');
legend('x','y','z');

figurec;
subplot(311);
plot(gps_time, abs(out_Q_matrix(:,1)),'b'); hold on;
plot(gps_time, abs(out_Q_matrix(:,2)),'r');
plot(gps_time, abs(out_Q_matrix(:,3)),'g');
xlabel('Time (s)');
ylabel('Q Att. Noise (rad)');
legend('roll','pitch','yaw');

subplot(312);
plot(gps_time, abs(out_Q_matrix(:,4)),'b'); hold on;
plot(gps_time, abs(out_Q_matrix(:,5)),'r');
plot(gps_time, abs(out_Q_matrix(:,6)),'g');
xlabel('Time (s)');
ylabel('Q Vel. Noise (m/s)');
legend('x', 'y', 'z');

subplot(313);
plot(gps_time, abs(out_Q_matrix(:,7)),'b'); hold on;
plot(gps_time, abs(out_Q_matrix(:,8)),'r');
plot(gps_time, abs(out_Q_matrix(:,9)),'g');
xlabel('Time (s)');
ylabel('Q Pos. Noise (m)');
legend('x', 'y', 'z');

figurec;
subplot(211);
plot(gps_time, abs(out_Q_matrix(:,10)),'b'); hold on;
plot(gps_time, abs(out_Q_matrix(:,11)),'r');
plot(gps_time, abs(out_Q_matrix(:,12)),'g');
xlabel('Time (s)');
ylabel('Q Accel-bias Noise (m/s^2)');
legend('x', 'y', 'z');

subplot(212);
plot(gps_time, abs(out_Q_matrix(:,13)),'b'); hold on;
plot(gps_time, abs(out_Q_matrix(:,14)),'r');
plot(gps_time, abs(out_Q_matrix(:,15)),'g');
xlabel('Time (s)');
ylabel('Q Gyro-bias Noise (rad/s)');
legend('x', 'y', 'z');