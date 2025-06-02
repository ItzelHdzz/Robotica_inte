% Inicialización
clear all; close all; clc

% Parámetros
L3 = 3; L4 = 2; L5 = 1;

% Ángulos
th1 = pi; th2 = pi/2; th3 = -pi/2; th4 = pi/2; th5 = 0; th6 = pi/8; th7 = 0;

% Transformaciones
H0 = SE3(rotx(th1), [0 0 0]);
H1 = SE3(rotx(-pi/2)*roty(pi/2), [0 0 0]);
H2 = SE3(roty(-pi/2)*rotz(-pi/2), [0 0 0]);
H3 = SE3(rotz(th3), [0 0 L3]);
H4 = SE3(rotz(th4)*rotx(-th4), [L4 0 0]);

% Pie (H5): Plantarflexión (hacia abajo)
R5 = roty(pi/2) * rotx(-pi/8);
H5 = SE3(R5, [L5 0 0]);

% H6: Inversión/Eversión
H6 = SE3(rotz(th6), [0 0 0]);

% H7: Ajuste de orientación final 
H7 = SE3(eye(3), [1 0 0]);

% Transformaciones acumuladas
H0_1 = H0 * H1;
H1_2 = H0_1 * H2;
H2_3 = H1_2 * H3;
H3_4 = H2_3 * H4;
H4_5 = H3_4 * H5;
H5_6 = H4_5 * H6;
H6_7 = H5_6 * H7;

% Posiciones
P0 = [0;0;0];
P1 = H0_1.t;
P2 = H1_2.t;
P3 = H2_3.t;
P4 = H3_4.t;
P5 = H4_5.t;
P6 = H5_6.t;
P7 = H6_7.t;

% Graficar trayectoria
x = [P0(1) P1(1) P2(1) P3(1) P4(1) P5(1) P6(1) P7(1)];
y = [P0(2) P1(2) P2(2) P3(2) P4(2) P5(2) P6(2) P7(2)];
z = [P0(3) P1(3) P2(3) P3(3) P4(3) P5(3) P6(3) P7(3)];

plot3(x, y, z, 'LineWidth', 2, 'Color', 'c'); hold on;
axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z'); view(3)
title('Cinemática Pierna Completa - 7 GDL')

% Mostrar tramas
trplot(H0, 'rgb')
pause; tranimate(H0, H0_1, 'rgb')
pause; tranimate(H0_1, H1_2, 'rgb')
pause; tranimate(H1_2, H2_3, 'rgb')
pause; tranimate(H2_3, H3_4, 'rgb')
pause; tranimate(H3_4, H4_5, 'rgb')
pause; tranimate(H4_5, H5_6, 'rgb')
pause; tranimate(H5_6, H6_7, 'rgb')
