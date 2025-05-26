clear
r_wheel = 0.3125; % [m]
vxw = -1:0.01:1; % [m/s]
w_wheel = (-10:0.01:10)'; % [rad/s]
slip_threshold = 0.1;
for i = 1:length(vxw)
  for j = 1:length(w_wheel)
    slip_ratio(j, i) = (w_wheel(j)*r_wheel - vxw(i)) / ...
          max([abs(w_wheel(j)*r_wheel), abs(vxw(i)), slip_threshold]);
  endfor
endfor

%% plot
clf
surf(vxw, w_wheel, slip_ratio)
xlabel("v{xw}")
ylabel("w_{wheel}")
zlabel("slip ratio")
