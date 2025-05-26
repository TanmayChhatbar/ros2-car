% vars
B = 0.7; % stiffness factor
C = 1.5; % shape factor
D = 1; % peak factor (basically mu)
E = 0.8; % curvature factor

% calc
slips = -30:0.1:30;
F = fcn_pacejka(slips, B, C, D, E);

% plot
plot(slips, F)
grid on
xlabel("slip (deg OR %)")
ylabel("Force (normalized)")
