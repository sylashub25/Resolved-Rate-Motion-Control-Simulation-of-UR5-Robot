function [qp, xerror, xp]=posicion_ur5(t,x)
disp(t);
[~, n]=size(t);
%Estados
q1=x(1);
q2=x(2);
q3=x(3);
q4=x(4);
q5=x(5);
q6=x(6);

%Vector de posición articular
q=[q1;     
   q2;
   q3;
   q4;
   q5;
   q6];
 
%% Trayectoria deseada en el espacio cartesiano
%Inicialización de vectores
xd=zeros(1,n);
yd=zeros(1,n);
% zd=zeros(1,n);
%% Rosa polar
%Parametros de la rosa polar
% w=1.35*pi;
r=0.2;
alfa=0.07;
beta=5;
%Dibujamos
xd(1,:)=0.3 + alfa + r*(cos(t))*(sin(beta*t));
yd(1,:)=0.3 + alfa + r*(sin(t))*(sin(beta*t));
zd=ones(1,n)*0.2;
%% Vector de posición cartesiana deseada
PosicionDeseada=[xd;yd;zd];

%% Orientación deseada, dada por los angulos de EULER XYZ
%Variable auxiliar para definir una trayecotoria para la orientacion como rotacion en el eje z
phi=ones(1,n)*(99*pi/100);
theta=ones(1,n)*(1/100);
psi=ones(1,n)*(1/100);
%Vector de angulos de Euler
wtd=[phi;theta;psi];
     
%Cinemática directa para conocer la posición actual
Ha=ur5fwdtrans(q, 6);
%Posiciones actuales en el espacio cartesiano
PosicionActual=Ha(1:3,4);
%Orientaciones actuales en el espacio cartesiano
wa=EULERXYZINV1(Ha(1:3,1:3));

%% Error de posición cartesiano y orientacion
xerror = [PosicionDeseada - PosicionActual; %Posición
                   wtd - wa;]; %Orientación
%% Matriz de ganancias para corregir el error de posición
%Estos valores de ganancias funcionas muy bien (no borrar)
% k1=85;
% k2=75;
% k3=65;
% k4=20;
% k5=15;
% k6=10;

%Establecemos los valores de las ganancias
k1=25;
k2=20;
k3=15;
k4=5;
k5=5;
k6=5;

%Matriz de ganancias
K=[ k1,0,0,0,0,0;
    0,k2,0,0,0,0;
    0,0,k3,0,0,0;
    0,0,0,k4,0,0;
    0,0,0,0,k5,0;
    0,0,0,0,0,k6];

%% Velocidades deseadas en posición cartesiana y orientación
% Flor de petalos
xdp(1,:)=-r*(sin(t))*(sin(beta*t)) + r*beta*(cos(t))*(cos(beta*t));
ydp(1,:)= r*(cos(t))*(sin(beta*t)) + r*beta*(sin(t))*(cos(beta*t));
zdp=zeros(1,n);
vcd = [xdp;ydp;zdp];
%% Vector de velocidad cartesiana deseada
phi_p = zeros(1,n);
theta_p = zeros(1,n);
psi_p = zeros(1,n);
vad = [phi_p;theta_p;psi_p];
xpref=[vcd;vad];
 %Jacobiano analítico del robot para posicion y orientacion actual
 Ja_xyz = ur5f_Ja_xyz(q,wa);
 
%Aqui podemos saturar tau
qp=Ja_xyz^(-1)*(xpref + K*xerror);           %Ecuacion en lazo cerrado
xp=[qp(1);qp(2);qp(3);qp(4);qp(5);qp(6)];
end