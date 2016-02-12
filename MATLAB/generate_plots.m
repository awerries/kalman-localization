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

% %% Compare ground truth to raw NovAtel data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figurec;
plot(gps_x, gps_y,'r--', 'LineWidth', 2);
xlabel('Easting (m)');
ylabel('Northing (m)');
hold on;
plot(ground_truth(:,1),ground_truth(:,2),'c');
legend('NovAtel','Applanix');
axis equal;

figurec;
subplot(311);
plot(nov_time, gps_x,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting (m)');
hold on; plot(nov_time, ground_truth(:,1),'c');
legend('NovAtel','Applanix');

subplot(312);
plot(nov_time, gps_y,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing (m)');
hold on; plot(nov_time, ground_truth(:,2),'c');
legend('NovAtel','Applanix','Location','SouthEast');

subplot(313);
plot(nov_time, gps_h,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing (m)');
hold on; plot(nov_time, ground_truth(:,3),'c');
legend('NovAtel','Applanix','Location','SouthEast');

%% Plot filter results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
roll = out_profile(:,8);
pitch = out_profile(:,9);
yaw = out_profile(:,10);
figurec;
subplot(311);
plot(filter_time, rad2deg(roll));
xlabel('Time (s)');
ylabel('Roll (deg)');
subplot(312);
plot(filter_time, rad2deg(pitch));
xlabel('Time (s)');
ylabel('Pitch (deg)');
subplot(313);
plot(filter_time, rad2deg(yaw));
xlabel('Time (s)');
ylabel('Yaw (deg)');

figurec;
plot(x, y,'r--', 'LineWidth', 2);
xlabel('Easting (m)');
ylabel('Northing (m)');
hold on;
plot(ground_truth(:,1),ground_truth(:,2),'c');
plot(gps_x, gps_y,'b-.', 'LineWidth', 2);
legend('Filtered','Applanix','NovAtel');
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
plot(filter_time, x-ground_truth_full(:,1),'r--', 'LineWidth', 2); hold on;
plot(nov_time, gps_x-ground_truth(:,1), ':g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting error (m)');
ylim([-10 10]);
plot(filter_time, zero_line, '--b');

subplot(312);
plot(filter_time, y-ground_truth_full(:,2),'r--', 'LineWidth', 2); hold on;
plot(nov_time, gps_y-ground_truth(:,2), ':g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing error (m)');
ylim([-10 10]);
plot(filter_time, zero_line, '--b');

subplot(313);
plot(filter_time, h-ground_truth_full(:,3),'r--', 'LineWidth', 2); hold on;
plot(nov_time, gps_h-ground_truth(:,3), ':g', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Altitude error (m)');
ylim([-10 10]);
plot(filter_time, zero_line, '--b');
