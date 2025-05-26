clear
clf

Tmax = 0.18;                          % [Nm]
Pmax = 40;                            % [W]
w_wheel = linspace(0, 16000/60*2*pi, 1e3); % [rad/s]
r_wheel = 0.03125;                    % [m]
gear_ratio = 15;
vx = w_wheel * r_wheel / gear_ratio;  % [m/s]

Tmaxcalc = min(Tmax, Pmax./w_wheel);

plot(vx, Tmaxcalc)
ylim([0 Tmax*1.5])
xlabel("vx (m/s)")
ylabel("Tmax (Nm)")
grid on

