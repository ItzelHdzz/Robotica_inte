%Limpieza de pantalla
clear all
close all
clc

%Calculamos las matrices de transformación homogénea


%H0=SE3;                             %Matriz identidad

%Rotacción θ1 sobre Z 
th0 = pi/2;
H00=SE3(roty(th0), [0 0 0]);
H0=SE3( rotx(th0), [0 0 0]);

%Rotacción θ1 sobre Z y trslación L1 sobre Z
L1= 0; th1 = -pi/2;
H1=SE3(rotx(th1), [0 0 L1]);

%Rotación de θ2 en X y traslación L2 en X
L2 = 5; th2 = 0;
H2=SE3(rotz(th2), [L2 0 0]);

%Rotación de θ3 en Z y traslación en X
L3 = 5; th3 = -pi/2;
H3=SE3(rotz(th3), [L3 0 0]);

%Rotación de θ3 en Z y traslación en X
L4 = 0; th4  = -pi/2;
H4=SE3(rotx(th4), [L4 0 0]);
H44=SE3(rotz(th4), [L4 0 0]);

H00_0 = H00*H0;
H0_1= H00_0*H1;
H1_2= H0_1*H2;
H2_3= H1_2*H3;
H3_4= H2_3*H4 
H44_4= H3_4*H44 %Matriz de transformación homogenea global de 4 a 0 

% Coordenadas de la estructura (segmentos del brazo robótico)
P0 = [0; 0; 0];                     % Base
P1 = H0_1.t;                        % Posición después de H1 (al aplicar H0*H1)
P2 = H1_2.t;                        % Posición después de H2
P3 = H2_3.t; 
P4 = H3_4.t;% Posición final

% Unir en vectores para graficar trayectoria del robot
x = [P0(1) P1(1) P2(1) P3(1) P4(1)];
y = [P0(2) P1(2) P2(2) P3(2) P4(2)];
z = [P0(3) P1(3) P2(3) P3(3) P4(3)];

plot3(x, y, z,'LineWidth', 2, 'Color', 'c'); 
hold on;

%Graficamos la trama absoluta o global 
trplot(H00_0,'rgb','axis', [-1 4 -1 6 -1 2])
% 
% %Realizamos una animación para la siguiente trama
 pause;
 tranimate(H00_0, H0_1,'rgb','axis', [-1 4 -1 6 -1 2])
% %Realizamos una animación para la siguiente trama
 pause;
 tranimate(H0_1, H1_2,'rgb','axis', [-1 4 -1 6 -1 2])
% % %Realizamos una animación para la siguiente trama
 pause;           
  tranimate(H1_2, H2_3,'rgb','axis', [-1 4 -1 6 -1 2])
  disp(H2_3)
% % %Realizamos una animación para la siguiente trama
 pause;           
  tranimate(H2_3, H3_4,'rgb','axis', [-1 4 -1 6 -1 2])
  disp(H3_4)
% % %Realizamos una animación para la siguiente trama
 pause;           
  tranimate(H3_4, H44_4,'rgb','axis', [-1 4 -1 6 -1 2])
  disp(H44_4)
% Ejes
axis equal; grid on; view(3)
xlabel('X'), ylabel('Y'), zlabel('Z')
title('Cadena Cinemática 3D')