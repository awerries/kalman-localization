
%% Compute error statistics on groundtruth vs NovAtel
distance = ((ground_truth(:,1)-gps_x).^2 + (ground_truth(:,2)-gps_y).^2).^0.5;
raw_rms = rms(distance);
raw_max = max(distance);
fprintf('\nRMS 2D Raw: %0.4fm\n',raw_rms)
fprintf('Max 2D Raw: %0.4fm\n',raw_max)

%% Compute error statistics on groundtruth vs filter output
xyz = out_profile(:,2:4);
llh = ecef2lla(xyz);
[x,y] = deg2utm(llh(:,1),llh(:,2));
x = x-min_x;
y = y-min_y;
h = -llh(:,3);

distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2).^0.5;
filter_rms = rms(distance);
filter_max = max(distance);
fprintf('RMS 2D Filter: %0.4fm, %0.2f%% change\n',filter_rms,(raw_rms-filter_rms)/raw_rms*100);
fprintf('Max 2D Filter: %0.4fm, %0.2f%% change\n',filter_max,(raw_max-filter_max)/raw_max*100);

%% 3D Stats
distance = ((ground_truth(:,1)-gps_x).^2 + (ground_truth(:,2)-gps_y).^2 + (ground_truth(:,3)-gps_h).^2).^0.5;
raw_rms = rms(distance);
raw_max = max(distance);
fprintf('\nRMS 3D Raw: %0.4fm\n',raw_rms)
fprintf('Max 3D Raw: %0.4fm\n',raw_max)
distance = ((ground_truth_full(:,1)-x).^2 + (ground_truth_full(:,2)-y).^2 + (ground_truth_full(:,3)-h).^2).^0.5;
filter_rms = rms(distance);
filter_max = max(distance);
fprintf('RMS 3D Filter: %0.4fm, %0.2f%% change\n',filter_rms,(raw_rms-filter_rms)/raw_rms*100);
fprintf('Max 3D Filter: %0.4fm, %0.2f%% change\n',filter_max,(raw_max-filter_max)/raw_max*100);


%% Rotation stats
