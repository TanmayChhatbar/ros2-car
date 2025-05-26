clear
% vars
B = 0.7; % stiffness factor
C = 1.5; % shape factor
D = 1; % peak factor (basically mu)
E = 0.8; % curvature factor
f = 100;

% calc
slip_angles = linspace(-10, 10, 100);   % [deg]
slip_ratios = linspace(-0.2, 0.2, 100); % [ratio]
[slip_angles, slip_ratios] = meshgrid(slip_angles, slip_ratios);
[Fx, Fy] = fcn_pacejka_combined(slip_angles, slip_ratios, B, C, D, E, f);

% plot
figure(1);
clf
surf(slip_angles, slip_ratios*100, Fx)
grid on
xlabel("slip angle (deg)")
ylabel("slip ratio (%)")
zlabel("Fx (normalized)")
title("Fx vs slips")

figure(2);
clf
surf(slip_angles, slip_ratios*100, Fy)
grid on
xlabel("slip angle (deg)")
ylabel("slip ratio (%)")
zlabel("Fy (normalized)")
title("Fy vs slips")

figure(3);
clf
surf(slip_angles, slip_ratios*100, sqrt(Fx.^2 + Fy.^2))
grid on
xlabel("slip angle (deg)")
ylabel("slip ratio (%)")
zlabel("Fnet (normalized)")
title("Fnet vs slips")
