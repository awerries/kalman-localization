% Code to grab data, parse, adjust timing, run the filters, and plot 
% Adam Werries 2016, see Apache 2.0 license.
close all; clear all;
datasets = {'oakland_oct24_meh', 'oakland_oct24_meh2',...
            'oakland_oct24_meh3', 'oakland_oct24_meh4'};
applanix = {'2015.10.24.18.57.07','2015.10.24.19.05.51',...
            '2015.10.24.19.11.42','2015.10.24.19.19.43'};
%% Import Applanix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ground truth import
app_times = zeros(length(applanix),2);
for i = 1:length(applanix)
    gt = csvread(sprintf('%s/VehicleState.csv',applanix{i}));
    t = gt(:,1);
    app_times(i,:) = [min(t), max(t)];
%     vel = gt(:,8:10);
%     speed = gt(:,21);
%     rot = gt(:,11:13);
%     accel = gt(:,14:16);
    xyz = gt(:,27:29);
    figurec;
    plot(xyz(:,2),xyz(:,1));
    title(applanix{i});
end

%% Import NovAtel data %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Format:  1     2                   3               4  5  6  7      8      
%         time, sol good? (1 or 0), WAAS? (1 or 0), x, y, z, sig_x, sig_y
%         9      10  11  12  13      14      15      16        17
%         sig_z, vx, vy, vz, sig_vx, sig_vy, sig_vz, num_sats, sol_sats
nov_times = zeros(length(datasets),2);
for i = 1:length(datasets)
    novatel = csvread(sprintf('%s/gps_log_parsed.txt',datasets{i}));
    nov_times(i,:) = [min(novatel(:,1)) max(novatel(:,1))];
    % Convert ECEF to lat-long-altitude
    lla = ecef2lla(novatel(:,4:6));
    % Convert latlon to UTM
    [gps_x,gps_y,utmzone] = deg2utm(lla(:,1),lla(:,2));
    gps_h = lla(:,3);
    figurec;
    plot(gps_x, gps_y);
    datasets{i} = strrep(datasets{i},'_','\_');
    title(datasets{i});
end

figurec;
plot(app_times(1,:), [1 1]);
hold all;
for i = 2:length(applanix)
    plot(app_times(i,:), [1 1]);
end
for i = 1:length(datasets)
    plot(nov_times(i,:), [2 2]);
end
ylim([0 3]);
legend([applanix datasets]);
