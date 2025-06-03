clear
filename = "../src/build/vehicle_sim_test.csv";
ds = loadcsv(filename);

% config = loadcsv("../src/build/vehicle_config_test.csv");

figure(1);
clf
subplot(3,3,1)
plot(ds.X, ds.Y)
hold on
grid on
% daspect([1 1 1])
xlabel("Position (m)")
ylabel("Position (m)")
ll = 10;
for i = round(linspace(1, length(ds.time), 8))
  % direction of motion
  % quiver(ds.X(i), ds.Y(i), ...
    % ds.vx(i)*cos(ds.yaw(i))-ds.vy(i)*sin(ds.yaw(i)), ...
    % ds.vy(i)*cos(ds.yaw(i))+ds.vx(i)*sin(ds.yaw(i)), 'k')

  % direction of heading
  quiver(ds.X(i), ds.Y(i), ll*cos(ds.yaw(i)), ll*sin(ds.yaw(i)), 'k')
endfor
hold off

s(1) = subplot(3,3,2);
plot(ds.time, ds.vx * 3.6, "DisplayName", "v_x")
hold on
plot(ds.time, ds.vy * 3.6, "DisplayName", "v_y")
xlabel("Time (s)")
ylabel("v (kph)")
legend("show")
grid on

s(end+1) = subplot(3,3,3);
plot(ds.time, ds.ax)
hold on
plot(ds.time, ds.ay)
legend("ax", "ay")
xlabel("Time (s)")
ylabel("acc (m/s^2)")
grid on

s(end+1) = subplot(3,3,4);
plot(ds.time, ds.yaw)
xlabel("Time (s)")
ylabel("yaw (rad)")
grid on

s(end+1) = subplot(3,3,5);
plot(ds.time, ds.w_yaw)
xlabel("Time (s)")
ylabel("yawrate (rad/s)")
grid on

s(end+1) = subplot(3,3,6);
plot(ds.time, ds.w_wheel_fl)
hold on
plot(ds.time, ds.w_wheel_fr)
plot(ds.time, ds.w_wheel_rl)
plot(ds.time, ds.w_wheel_rr)
xlabel("Time (s)")
ylabel("wheel speeds (rad/s)")
legend("fl", "fr", "rl", "rr")
grid on

s(end+1) = subplot(3,3,7);
plot(ds.time, ds.Fx_wheel_fl)
hold on
plot(ds.time, ds.Fx_wheel_fr)
plot(ds.time, ds.Fx_wheel_rl)
plot(ds.time, ds.Fx_wheel_rr)
legend("fl", "fr", "rl", "rr")
xlabel("Time (s)")
ylabel("F_x (N)")
grid on

s(end+1) = subplot(3,3,8);
plot(ds.time, ds.Fy_wheel_fl)
hold on
plot(ds.time, ds.Fy_wheel_fr)
plot(ds.time, ds.Fy_wheel_rl)
plot(ds.time, ds.Fy_wheel_rr)
legend("fl", "fr", "rl", "rr")
xlabel("Time (s)")
ylabel("F_y (N)")
grid on

s(end+1) = subplot(3,3,9);
plot(ds.time ,ds.steering)
xlabel("Time (s)")
ylabel("Steering (rad)")
grid on

linkaxes(s, 'x');
%% animate
% animate_car(config, ds, figure(2));
