%% Define vehicle
R = 0.1; %Wheel radius [m]
L = 0.5; %Wheel base [m]
dd = DifferentialDrive(R,L);


%% Simulation parameters
op = 1; 

if op == 1
    sampleTime = 0.01;
    tVec = 0:sampleTime:59;
    
    initPose = [22.3; 3.6; deg2rad(180)];
    pose = zeros (3, numel(tVec));
    pose(:,1) = initPose;
    
    %Define waypoints
    waypoints = [22.3, 3.6; 18.3, 3.6; 14.5, 7.7; 11, 11; 5.4, 25.8; 3.4, 22.3; 0, 22.3; -3.5, 26; -3.5, 21; -11.2, 18.41; -11.2, 14.6; -18.3, 14.6; -18.4, 10; -14.6, 3.5; -3.6, 3.7; 0, 0; 0, -3.7; 0, -11; 3.6, -18.4];
    %% Pure pursuit controller
    controller = controllerPurePursuit;
    controller.Waypoints = waypoints;
    controller.LookaheadDistance = 0.35;
    controller.DesiredLinearVelocity = 2;
    controller.MaxAngularVelocity = 2.8;

elseif op == 2
    sampleTime = 0.02;
    tVec = 0:sampleTime:126;
    
    initPose = [4; 1;  deg2rad(180)];
    pose = zeros (3, numel(tVec));
    pose(:,1) = initPose;
    
    %Define waypoints
    waypoints = [4, 1; 2, 3; 0, 3; 2, 1; 6, 1; 8, 3; 6, 3; 4, 1; 4, 5; 6, 3; 6, 5; 8, 5; 6, 7; 8, 9; 6, 9; 6, 11; 4, 9; 2, 11; 2, 9; 0, 9; 2, 7; 0, 5; 2, 5; 2, 3; 4, 5];
    %% Pure pursuit controller
    controller = controllerPurePursuit;
    controller.Waypoints = waypoints;
    controller.LookaheadDistance = 0.35;
    controller.DesiredLinearVelocity = 0.5;
    controller.MaxAngularVelocity = 1.5;

elseif op == 3
    sampleTime = 0.02;
    tVec = 0:sampleTime:91;
    
    initPose = [10; 9;  deg2rad(180)];
    pose = zeros (3, numel(tVec));
    pose(:,1) = initPose;
    
    %Define waypoints
    waypoints = [10,9; 4,9; 6,11; 8,11; 10,9; 7,5; 6,6; 7,5; 8,5; 7,5; 7.86,6.14; 7,7; 5,7; 3, 5; 3,3; 5,1; 7,1; 9,3; 9,5; 7,7];
    %% Pure pursuit controller
    controller = controllerPurePursuit;
    controller.Waypoints = waypoints;
    controller.LookaheadDistance = 0.35;
    controller.DesiredLinearVelocity = 0.5;
    controller.MaxAngularVelocity = 1.5;
end 

%Create vizualizaer
viz = Visualizer2D;
viz.hasWaypoints = true ;


%% Simulation loop
close all 
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    %comentario
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    %Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w];
    vel = bodyToWorld(velB,pose(:, idx-1));

    %si
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    %upgrade
    viz(pose(:,idx), waypoints);
    waitfor(r);
end
%% Gráficas de velocidad lineal y angular

% Crear vector de tiempo ajustado (por si acaso)
tVec_plot = tVec(1:numel(pose(1,:))-1);  % Aseguramos longitud coherente

% Inicializar vectores de velocidades
v_log = zeros(1, numel(tVec_plot));
w_log = zeros(1, numel(tVec_plot));

% Volver a simular velocidades para graficar
for idx = 1:numel(tVec_plot)
    [v_log(idx), w_log(idx)] = controller(pose(:,idx));
end

% Gráficas
graph = figure;
set(graph,'position',get(0,'ScreenSize')); % Pantalla completa

subplot(2,1,1)
plot(tVec_plot, v_log, 'g', 'LineWidth', 2), grid on
xlabel('Tiempo [s]')
ylabel('v (m/s)')
legend('Velocidad Lineal')

subplot(2,1,2)
plot(tVec_plot, w_log, 'b', 'LineWidth', 2), grid on
xlabel('Tiempo [s]')
ylabel('w (rad/s)')
legend('Velocidad Angular')
