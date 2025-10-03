function q = EULERXYZINV1(R)
[rows, cols] = size(R);
if ((rows ~= 3) || (cols ~= 3))
  error('EULERXYZINV requiere una matriz 3x3 por favor checar las dimensiones');
end

theta = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
psi = atan2(R(2,1)/cos(theta), R(1,1)/cos(theta));
phi = atan2(R(3,2)/cos(theta), R(3,3)/cos(theta));

q=[phi;theta;psi];

