%% Define vehicle
R = 0.1; %Wheel radius [m]
L = 0.5; %Wheel base [m]
dd = DifferentialDrive(R,L);


%% Simulation parameters
op = 1; 

if op == 1
    sampleTime = 0.02;
    tVec = 0:sampleTime:82;
    
    initPose = [4; 4; deg2rad(180)];
    pose = zeros (3, numel(tVec));
    pose(:,1) = initPose;
    
    %Define waypoints
    waypoints = [4,4; -8,10; 8,-1; -7,-6; 0,5; 3,0; 3,-5; 0,0];
elseif op == 2
    sampleTime = 0.02;
    tVec = 0:sampleTime:50;
    
    initPose = [2; 5;  deg2rad(180)];
    pose = zeros (3, numel(tVec));
    pose(:,1) = initPose;
    
    %Define waypoints
    waypoints = [2,5; -5,3; -5,-2; 2,-5; 5,2; -3,2; -4,-4; 4,-3];

elseif op == 3
    sampleTime = 0.02;
    tVec = 0:sampleTime:38.5;
    
    initPose = [-3; 4; 0];
    pose = zeros (3, numel(tVec));
    pose(:,1) = initPose;
    
    %Define waypoints
    waypoints = [-3,4; 3,3; 1,-3; -1,-1; 1,4; -3,-4; 2,-1];
end 

%Create vizualizaer
viz = Visualizer2D;
viz.hasWaypoints = true ;


%% Pure pursuit controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity = 2;


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
