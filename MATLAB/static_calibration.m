
if ~exist('last_lowcost_dir', 'var') || ~ischar(last_lowcost_dir)
   last_lowcost_dir = pwd; 
end
static_mean = zeros(9,3);
for i = 1:9
    [imu_file, imu_path] = uigetfile(text_files, ['Select accelerometer calibration file ' num2str(i)], last_lowcost_dir);
    last_lowcost_dir = imu_path;
    imu = csvread([imu_path imu_file]);
    mean(imu(:,3:5))
    max(imu(:,3:5))
    min(imu(:,3:5))
    static_mean(i,:) = mean(imu(:,3:5));
    
end

[M,B] = CalibAccel(static_mean)