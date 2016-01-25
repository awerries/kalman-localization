%% Compare ground truth to raw NovAtel data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
xyz = out_profile(:,2:4);
llh = ecef2lla(xyz);
[x,y] = deg2utm(llh(:,1),llh(:,2));
x = x-min_x;
y = y-min_y;
h = -llh(:,3)-1.5;
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
subplot(211);
plot(filter_time, x,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting (m)');
hold on; plot(filter_time, ground_truth_full(:,1),'c');
legend('Filtered','Applanix');

subplot(212);
plot(filter_time, y,'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing (m)');
hold on; plot(filter_time, ground_truth_full(:,2),'c');
legend('Filtered','Applanix','Location','SouthEast');

% subplot(313);
% plot(imu_time, h,'r--', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Height (m)');
% hold on; plot(imu_time, ground_truth_full(:,3),'c');
% legend('Filtered','Applanix','Location','SouthEast');
%% Compute error statistics on groundtruth vs NovAtel %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
distance = ((ground_truth(:,1)-gps_x).^2 + (ground_truth(:,2)-gps_y).^2).^0.5;
fprintf('\nRMS Raw:%f\n',rms(distance))
fprintf('Max Raw:%f\n',max(distance))

%% Compute error statistics on groundtruth vs filter output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
fprintf('\nRMS Filter:%f\n',rms(distance))
fprintf('Max Filter:%f\n',max(distance))
figurec;
subplot(211);
plot(filter_time, x-ground_truth_full(:,1),'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Easting error (m)');
ylim([-10 10])
subplot(212);
plot(filter_time, y-ground_truth_full(:,2),'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Northing error (m)');
ylim([-10 10])
