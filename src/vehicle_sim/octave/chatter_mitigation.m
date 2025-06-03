v0 = -0.19111;
F = -1;
m = 1;
dt = 0.001;
t = 0:dt:0.5;

v = t*0;
a = v;
x = v;
v(1) = v0;
% simulate
for i = 2:length(t)
  a(i) = F/m*sign(v(i-1));
  if abs(v(i-1)) <= -sign(v(i-1))*a(i)*dt
    a(i) = -v(i-1)/dt;
  endif
  v(i) = v(i-1) + a(i)*dt;
  x(i) = x(i-1) + v(i)*dt;
endfor

% plot
subplot(3,1,1)
plot(t, a)
subplot(3,1,2)
plot(t, v, '.')
subplot(313)
plot(t, x)
