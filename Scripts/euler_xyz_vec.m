function [R] = euler_xyz_vec(AE)
%Esta función recibe la orientacion del robot dada por los angulos de euler XZY que corresponde con los angulos RPY un forma vectorial,
%es decir para una trayectoria dada AE es un vector 3 x 1 x n
phi = AE(1,1,:); %Angulo YAW,   guiñada giro alrededor del eje x
theta=AE(2,1,:); %Angulo PITCH, cabeceo giro alrededor del eje y
psi = AE(3,1,:); %Angulo ROLL,  alabeo  giro alrededor del eje z

% Se verifica la dimensión de la entrada
[rows, cols, ~] = size(AE);
if ((rows ~= 3) || (cols ~= 1)) 
  error('euler_xyz requiere un vector 3 x 1 x n, por favor checa tus dimensiones');
end

%Se premultiplican por ser rotaciones absolutas, es decir se hacen con respecto a los ejes fijos x0, y0, z0
% R = ROTz(psi)*ROTy(theta)*ROTx(phi);
R=[ cos(psi).*cos(theta), cos(psi).*sin(phi).*sin(theta) - cos(phi).*sin(psi), sin(phi).*sin(psi) + cos(phi).*cos(psi).*sin(theta);
    cos(theta).*sin(psi), cos(phi).*cos(psi) + sin(phi).*sin(psi).*sin(theta), cos(phi).*sin(psi).*sin(theta) - cos(psi).*sin(phi);
             -sin(theta),                                cos(theta).*sin(phi),                               cos(phi).*cos(theta)]; 
end



