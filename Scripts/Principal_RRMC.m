%% Inicio de programa 
clc;clearvars;close all;format short
ti=0;h=0.0025;tf=10;
t=ti:h:tf;%Vector de tiempo
[m, n] = size(t);
ci=[0 -pi/2 -pi/2 -pi/2 pi/2 0]; %Condiciones iniciales
opciones=odeset('RelTol',1e-06,'AbsTol',1e-06,'InitialStep',h,'MaxStep',h); %ajustar parametros ode45
%Solución numérica del sistema robot 
[t,x]=ode45('posicion_ur5',t,ci,opciones);  
q1=x(:,1);
q2=x(:,2);
q3=x(:,3);
q4=x(:,4);
q5=x(:,5);
q6=x(:,6);

%% Trayectoria deseada para gráficar
%% Flor de petalos
w=1.35*pi;
r=0.2;
alfa=0.07;
beta=5;

x=0.3 + alfa + r*(cos(t)).*(sin(beta*t));
y=0.3 + alfa + r*(sin(t)).*(sin(beta*t));
z=0.2*ones(n,1);

%% Trayectoria cartesiana obtenida
q=[q1, q2, q3, q4, q5, q6];
G1s=zeros(4,4,n);
x0_PD=zeros(n,1);
y0_PD=zeros(n,1);
z0_PD=zeros(n,1);

for k=1:n
 G1s(:,:,k) = ur5fwdtrans(  q(k,:), 6 );
 x0_PD(k) = G1s(1,4,k);
 y0_PD(k) = G1s(2,4,k);
 z0_PD(k) = G1s(3,4,k);
end

%% Orientación obtenida
zeta = zeros(3,1,n);
phi=zeros(n,1);
theta=zeros(n,1);
psi=zeros(n,1);
for k=1:n
    zeta(:,:,k)=EULERXYZINV1(G1s(1:3,1:3,k));
    phi(k) = zeta(1,1,k);
    theta(k) = zeta(2,1,k);
    psi(k) = zeta(3,1,k);
end

%% Variables para errores de velocidad
x_e=zeros(n,m);
y_e=zeros(n,m);
z_e=zeros(n,m);
phi_e=zeros(n,m);
theta_e=zeros(n,m);
psi_e=zeros(n,m);

qp1=zeros(n,m);
qp2=zeros(n,m);
qp3=zeros(n,m);
qp4=zeros(n,m);
qp5=zeros(n,m);
qp6=zeros(n,m);

for k=1:n
   [qp, xerror, xp]=posicion_ur5(t(k), [q1(k,1);q2(k,1);q3(k,1);q4(k,1);q5(k,1);q6(k,1)]);
    
   x_e(k,1)=xerror(1,1);
   y_e(k,1)=xerror(2,1);
   z_e(k,1)=xerror(3,1);
   phi_e(k,1)=xerror(4,1);
   theta_e(k,1)=xerror(5,1);
   psi_e(k,1)=xerror(6,1);
   
    qp1(k,1)=qp(1,1);
    qp2(k,1)=qp(2,1);
    qp3(k,1)=qp(3,1);
    qp4(k,1)=qp(4,1);
    qp5(k,1)=qp(5,1);
    qp6(k,1)=qp(6,1);   
end
qp=[qp1,qp2,qp3,qp4,qp5,qp6];
xerror=[x_e,y_e,z_e,phi_e,theta_e,psi_e];

%% Area de gráficas
%Posiciones articulares
figure(1)
plot(t, q, 'LineWidth',1.3);
title('Posiciones articulares')
xlabel('Tiempo (s)')
ylabel('Posiciones articulares (rad)')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');

%Velocidades articulares
figure(2)
plot(t, qp, 'LineWidth',1.3);
title('Velocidades articulares')
xlabel('Tiempo (s)')
ylabel('Velocidades articulares (rad/s)')
legend('qp1', 'qp2', 'qp3', 'qp4', 'qp5', 'qp6');

%Orientaciones del efector final
figure(3)
plot(t, phi, t, theta, t, psi, 'LineWidth',1.3);
title('Orientacion del efector final')
xlabel('Tiempo (s)')
ylabel('q (rad)')
legend('\phi', '\theta','\psi');

%Trayectoria realizada en el espacio cartesiano
figure(4)
plot3(x,y,z,'x',x0_PD,y0_PD,z0_PD,'.', 'Linewidth',2);
title('Trayectoria realizada en el espacio cartesiano')
xlabel('Eje x (m)')
ylabel('Eje y (m)')
zlabel('Eje z (m)')
legend({'Trayectoria deseada', 'Trayectoria obtenida'})

figure(5)
plot(t, xerror(:,1:3), 'LineWidth',1.3);
title('Errores de posición en espacio cartesiano')
xlabel('Tiempo (s)')
ylabel('Errores de posición (m)')
legend('x_e', 'y_e', 'z_e');

figure(6)
plot(t, xerror(:,4:6), 'LineWidth',1.3);
title('Errores de orientacion')
xlabel('Tiempo (s)')
ylabel('Errores de orientación (rad)')
legend('phi_e', 'theta_e', 'psi_e');
%% Inicializamos el robot ur5 de la biblioteca Peter Corke
mdl_ur5

%% Inicializamos al robot UR5 de la biblioteca de Peter Corke
 
figure(7)
plot3(x,y,z,'x',x0_PD,y0_PD,z0_PD,'.', 'Linewidth',2);
title('Trayectoria realizada en el espacio cartesiano')
xlabel('Eje x (m)')
ylabel('Eje y (m)')
zlabel('Eje z (m)')
legend({'Trayectoria deseada', 'Trayectoria obtenida'})
ur5.plot(q)