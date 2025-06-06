clear
clc

filename = "../src/control_map_nlopt.csv";
t = csvread(filename);

% get data from table
t = t(2:end, :); % remove header row
vx_targets = t(:, 1);
vyvx_ratios = t(:, 2);
vy_targets = vyvx_ratios .* vx_targets;
scores = t(:, 3);
wheel_speeds = t(:, 4);
steering_angles = t(:, 5);
yaw_rates = t(:, 6);

% make grid for each
m = length(unique(vx_targets));
n = length(vx_targets) / m;
vx_targets = reshape(vx_targets, m, n);
vyvx_ratios = reshape(vyvx_ratios, m, n);
vy_targets = reshape(vy_targets, m, n);
scores = reshape(scores, m, n);
wheel_speeds = reshape(wheel_speeds, m, n);
steering_angles = reshape(steering_angles, m, n);
yaw_rates = reshape(yaw_rates, m, n);
wheel_speed_ratios = wheel_speeds ./ (vx_targets / 0.03125);

% create 3D plot for each field
figure(1);
subplot(221);
surf(vx_targets, vyvx_ratios, scores);
xlabel('v_x [m/s]');
ylabel('v_y/v_x [-]');
zlabel('Score [-]');
% title('Scores vs Vx and Vy Targets');

subplot(222);
surf(vx_targets, vyvx_ratios, wheel_speed_ratios);
zlabel('Wheel speed / Rolling wheel speed [-]');
% zlabel('Wheel speed [rad/s]');
xlabel('v_x [m/s]');
ylabel('v_y/v_x [-]');
% title('Wheel speeds vs v_x, v_y');

subplot(223);
surf(vx_targets, vyvx_ratios, steering_angles);
xlabel('v_x [m/s]');
ylabel('v_y/v_x [-]');
zlabel('Steering angle [rad]');
% title('Steering angles vs v_x, v_y');

subplot(224);
surf(vx_targets, vyvx_ratios, yaw_rates);
xlabel('v_x [m/s]');
ylabel('v_y/v_x [-]');
zlabel('Yaw rate [rad/s]');
% title('Yaw rates vs v_x, v_y');

% linkprop([subplot(221), subplot(222), subplot(223), subplot(224)], {'CameraPosition', 'CameraUpVector', 'CameraTarget', 'CameraViewAngle'});

