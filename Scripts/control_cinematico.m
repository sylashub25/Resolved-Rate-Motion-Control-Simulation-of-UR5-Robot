function qp=control_cinematico(q,t)

%Posiciones articulares
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);

%Parámetros geometricos del robot(metros)
 d1=0.089159;
 a2=0.425;
 a3=0.39225;
 d4=0.10915;
 d5=0.09465;
 d6=0.0823;
 
%Jacobiano del manipulador
J=[    d4*cos(q1) + d6*(cos(q1)*cos(q5) + 1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + 1.0*a2*cos(q2)*sin(q1) - d5*sin(q2 + q3 + q4)*sin(q1) + 1.0*a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3),     1.0*a2*cos(q1)*sin(q2) + d5*cos(q2 + q3 + q4)*cos(q1) + a3*cos(q1)*cos(q2)*sin(q3) + 1.0*a3*cos(q1)*cos(q3)*sin(q2) + 1.0*d6*sin(q2 + q3 + q4)*cos(q1)*sin(q5),     d5*cos(q2 + q3 + q4)*cos(q1) + 1.0*a3*cos(q1)*cos(q2)*sin(q3) + a3*cos(q1)*cos(q3)*sin(q2) + 1.0*d6*sin(q2 + q3 + q4)*cos(q1)*sin(q5),     d5*cos(q2 + q3 + q4)*cos(q1) + 1.0*d6*sin(q2 + q3 + q4)*cos(q1)*sin(q5),                                                                             -d6*(sin(q1)*sin(q5) + 1.0*cos(q2 + q3 + q4)*cos(q1)*cos(q5)),                                                                                                                                                                       0;
   1.0*d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + 1.0*d4*sin(q1) - 1.0*a2*cos(q1)*cos(q2) + d5*sin(q2 + q3 + q4)*cos(q1) - 1.0*a3*cos(q1)*cos(q2)*cos(q3) + a3*cos(q1)*sin(q2)*sin(q3),     1.0*a2*sin(q1)*sin(q2) + d5*cos(q2 + q3 + q4)*sin(q1) + a3*cos(q2)*sin(q1)*sin(q3) + 1.0*a3*cos(q3)*sin(q1)*sin(q2) + 1.0*d6*sin(q2 + q3 + q4)*sin(q1)*sin(q5),     d5*cos(q2 + q3 + q4)*sin(q1) + 1.0*a3*cos(q2)*sin(q1)*sin(q3) + a3*cos(q3)*sin(q1)*sin(q2) + 1.0*d6*sin(q2 + q3 + q4)*sin(q1)*sin(q5),     d5*cos(q2 + q3 + q4)*sin(q1) + 1.0*d6*sin(q2 + q3 + q4)*sin(q1)*sin(q5),                                                                              1.0*d6*(cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1)),                                                                                                                                                                       0;
                                                                                                                                                                                                     0,                                                                                 d5*sin(q2 + q3 + q4) - a2*cos(q2) - a3*cos(q2 + q3) - d6*cos(q2 + q3 + q4)*sin(q5),                                                                     d5*sin(q2 + q3 + q4) - a3*cos(q2 + q3) - d6*cos(q2 + q3 + q4)*sin(q5),                         d5*sin(q2 + q3 + q4) - d6*cos(q2 + q3 + q4)*sin(q5),                                                                                                             -d6*sin(q2 + q3 + q4)*cos(q5),                                                                                                                                                                       0;
                                                                                                                                                                                                     0,                                                                                                                                                            sin(q1),                                                                                                                                   sin(q1),                                                                     sin(q1),  -cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)),  cos(q5)*sin(q1) + sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));                                                                                                                                                                                                   0,                                                                                                                                                           -cos(q1),                                                                                                                                  -cos(q1),                                                                    -cos(q1),   sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)), -cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)));                                                                                                                                                                                                    1,                                                                                                                                                                  0,                                                                                                                                         0,                                                                           0,                                       cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)),                                                          sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))];                                                                                                                                                                                                                                                                                                                                                                                        
%Cinemática directa
x0=d6*(cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + d4*sin(q1) + a2*cos(q1)*cos(q2) + d5*sin(q2 + q3 + q4)*cos(q1) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3);
y0=a2*cos(q2)*sin(q1) - d4*cos(q1) - d6*(cos(q1)*cos(q5) + cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + d5*sin(q2 + q3 + q4)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3);
z0=d1 + a3*sin(q2 + q3) + a2*sin(q2) - d5*cos(q2 + q3 + q4) - d6*sin(q2 + q3 + q4)*sin(q5);


%Trayectoria deseada
xd=1.3*cos(t);
yd=1.2*sin(t);
zd=0.3;

%Ganancias
K=[0.3,0,0;0,0.2,0;0,0,0.1]
qp=J^(-1)*([xdp;ydp;zdp] + K*(xd-x));


end