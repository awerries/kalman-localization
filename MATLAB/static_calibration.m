
%% Static Calibration of Accelerometer in 9 poses
if ~exist('last_lowcost_dir', 'var') || ~ischar(last_lowcost_dir)
   last_lowcost_dir = pwd; 
end
rotation = [1   0         0
            0   cosd(180) -sind(180)
            0   sind(180) cosd(180)];
static_mean = zeros(9,3);
for i = 1:9
    [imu_file, imu_path] = uigetfile(text_files, ['Select accelerometer calibration file ' num2str(i)], last_lowcost_dir);
    last_lowcost_dir = imu_path;
    imu = csvread([imu_path imu_file]);
    imu = imu(:,2:end);
    imu(:,2:4) = (rotation*imu(:,2:4)')';
    imu(:,5:7) = (rotation*imu(:,5:7)')';
    fprintf('Mean: %f, ', mean(imu(:,2:4)));
    static_mean(i,:) = mean(imu(:,2:4));
    fprintf('\n');
end

[M,B] = CalibAccel(static_mean);

%% Leveling and gyrocompassing for determining initial attitude
[imu_file, imu_path] = uigetfile(text_files, 'Select leveling IMU file', last_lowcost_dir);
imu_leveling = csvread([imu_path imu_file]);
% only using end_time here
imu_leveling = imu_leveling(:,2:end);
% convert g's to m/s^2 using local gravity estimate
imu_leveling(:,2:4) = imu_leveling(:,2:4)*9.80097;
% rotate sensor readings 180 deg about the x-axis
imu_leveling(:,2:4) = (rotation*imu_leveling(:,2:4)')';
imu_leveling(:,5:7) = (rotation*imu_leveling(:,5:7)')';
% apply accelerometer static calibration
imu_leveling(:,2:4) = (M*(imu_leveling(:,2:4)' - repmat(B,[1 size(imu_leveling,1)])))';
% take mean, use for determining attitude
accel_mean = mean(imu_leveling(:,2:4));
gyro_mean = mean(imu_leveling(:,5:7));
roll = atan2(-accel_mean(2), -accel_mean(3));
pitch = atan(accel_mean(1) / sqrt(accel_mean(2)^2 + accel_mean(3)^2));
yaw = atan2(-gyro_mean(2)*cos(roll) + gyro_mean(3)*sin(roll),...
              gyro_mean(1)*cos(pitch) + gyro_mean(2)*sin(roll)*sin(pitch) + gyro_mean(3)*cos(roll)*sin(pitch));
          
fprintf('Radians Roll: %f, Pitch: %f, Yaw: %f\n', roll, pitch, yaw);
fprintf('Degrees Roll: %f, Pitch: %f, Yaw: %f\n', rad2deg(roll), rad2deg(pitch), rad2deg(yaw));

