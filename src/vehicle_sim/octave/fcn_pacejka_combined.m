function [Fx, Fy] = fcn_pacejka_combined(slip_angle, slip_ratio, B, C, D, E, f)
  slip_net = sqrt(slip_angle.^2 + (slip_ratio*f).^2);
  Fnet = D*sin(C*atan(B.*slip_net-E*(B.*slip_net-atan(B.*slip_net))));

  Fx = Fnet .* slip_ratio * f ./ slip_net;
  Fy = Fnet .* slip_angle ./ slip_net;

  Fx(abs(Fnet)<eps) = 0;
  Fy(abs(Fnet)<eps) = 0;
endfunction
