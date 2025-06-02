%Limpieza de pantalla
clear all
close all
clc

%Calculamos las matrices de transformación homogénea
H0=SE3;                             %Matriz identidad
H1=SE3(rotz(pi), [3 0 0]);
H2=SE3(roty(-pi/2), [0 0 2]);
H3=SE3(rotz(-pi), [0 0 0])

H0_1= H0*H1;
H1_2= H0_1*H2;
H2_3= H1_2*H3 %Matriz de transformación homogenea global de 3 a 0 

% Coordenadas de la estructura (segmentos del brazo robótico)
P0 = [0; 0; 0];                     % Base
P1 = H0_1.t;                        % Posición después de H1 (al aplicar H0*H1)
P2 = H1_2.t;                        % Posición después de H2
P3 = H2_3.t;                        % Posición final

% Unir en vectores para graficar trayectoria del robot
x = [P0(1) P1(1) P2(1) P3(1)];
y = [P0(2) P1(2) P2(2) P3(2)];
z = [P0(3) P1(3) P2(3) P3(3)];

plot3(x, y, z,'LineWidth', 2, 'Color', 'c'); 
hold on;

%Graficamos la trama absoluta o global 
trplot(H0,'rgb','axis', [-1 4 -1 6 -1 2])
% 
% %Realizamos una animación para la siguiente trama
 pause;
 tranimate(H0, H0_1,'rgb','axis', [-1 4 -1 6 -1 2])
% %Realizamos una animación para la siguiente trama
 pause;
 tranimate(H0_1, H1_2,'rgb','axis', [-1 4 -1 6 -1 2])
% % %Realizamos una animación para la siguiente trama
 pause;           
  tranimate(H1_2, H2_3,'rgb','axis', [-1 4 -1 6 -1 2])
  disp(H2_3)