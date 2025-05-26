function F = fcn_pacejka(slip, B, C, D, E)
F = D*sin(C*atan(B*slip-E*(B*slip-atan(B*slip))));
endfunction
