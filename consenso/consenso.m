%% Limpieza del entorno
clear all
close all
clc

t = 0;
dt  = 0.01;
tf = 100;

distance = 1;
P_d1_L = [0; distance/2; 0];
P_d2_L = [0; -distance/2; 0];

Pd = [1; 0];

PL = [0; 0; 0];
P1 = [0; 1; pi/2];
P2 = [0; -1; 0];

% Nodo
node = ros2node('/velocity');

% Robot 1
velPub = ros2publisher(node, '/tb3_1/cmd_vel', 'geometry_msgs/Twist');
velSub = ros2subscriber(node, '/tb3_1/cmd_vel', 'geometry_msgs/Twist');

% Robot 2
velPub2 = ros2publisher(node, '/tb3_0/cmd_vel', 'geometry_msgs/Twist');
velSub2 = ros2subscriber(node, '/tb3_0/cmd_vel', 'geometry_msgs/Twist');

tic
while t < tf

    R_L = rotMatrix(PL(3));

    % Posiciones deseadas
    P_d1_I = PL + R_L * P_d1_L;
    P_d2_I = PL + R_L * P_d2_L;

    %% Actualizacion de posiciones
    P1 = updatePosition(P1, P_d1_I, dt, velPub); % Robot 1
    % P2 = updatePosition(P2, P_d2_I, dt, velPub2); % Robot 2
    PL = updatePositionLeader(PL, Pd,  dt); % Lider virtual
    
    dt = toc;
    tic;
    t = t + dt;
       
    % Visualización
    visualize(Pd, PL, P1, P2);
end

function P = updatePosition(P, Pd, dt, pub)
    [V, w] = calculateVelocities(P, Pd);
    [xp, yp, thetap] = getVelocityComponents(V, w, P(3));

    sendVelocitiesToRobot(V, w, pub);

    P(1) = P(1) + xp * dt;
    P(2) = P(2) + yp * dt;
    P(3) = P(3) + thetap * dt;

end

function P = updatePositionLeader(P, Pd, dt)
    [V, w] = calculateVelocities(P, Pd);
    [xp, yp, thetap] = getVelocityComponents(V, w, P(3));

    P(1) = P(1) + xp * dt;
    P(2) = P(2) + yp * dt;
    P(3) = P(3) + thetap * dt;
end

function R = rotMatrix(theta)
    R = [cos(theta) -sin(theta) 0;
         sin(theta) cos(theta) 0;
         0       0       1];
end

function [thetae, Pe] = getErrors(P, Pd)
    Pe = Pd(1:2) - P(1:2);
    thetad = atan2(Pd(2) - P(2), Pd(1) - P(1));
    thetae = atan2(sin(thetad - P(3)), cos(thetad - P(3)));
end

function [V, w] = getVelocities(d, thetae)
    [V, w] = control(d, thetae);
    vMax = 0.18;
    wMax = 2.7;
    V = saturate(V, vMax);
    w = saturate(w, wMax);
end

function [xp, yp, thetap] = getVelocityComponents(V, w, theta)
    xp = V * cos(theta);
    yp = V * sin(theta);
    thetap = w;
end

function [thetae, d] = applyReverse(thetae, d)
    if abs(thetae) > pi/2
        d = -d;
        thetae = thetae + sign(thetae) * -pi;
    end
end

function [V, w] = control(d, thetae)
    kpd = 1;
    kpr = 1;
    V = kpd * d;
    w = kpr * thetae;
end

function V = saturate(c, vMax)
     V = vMax * tanh(c/vMax);
end

function [V, w] = calculateVelocities(P, Pd)
    [thetae, Pe] = getErrors(P, Pd);
    d = norm(Pe);
    [thetae, d] = applyReverse(thetae, d);
    [V, w] = getVelocities(d, thetae);
    if (abs(d) < 0.05)
        V = 0;
        w = 0;
    end
end

function sendVelocitiesToRobot(V, w, pub)
    velocityMsg = ros2message("geometry_msgs/Twist");
    velocityMsg.linear.x = V;
    velocityMsg.linear.y = 0;
    velocityMsg.angular.z = w;
    send(pub, velocityMsg);
    clear velocityMsg;
end

function visualize(Pd, PL, P1, P2)
    figure(1)
    scatter(PL(1), PL(2), 'filled', 'blue');
    hold on
    scatter(P1(1), P1(2), 'filled', 'red');
    scatter(P2(1), P2(2), 'filled', 'green');
    scatter(Pd(1), Pd(2), '.black');
    plot([PL(1), PL(1) + 0.1 * cos(PL(3))],[PL(2), PL(2) + 0.1 * sin(PL(3))],'blue','LineWidth', 2);
    plot([P1(1), P1(1) + 0.1 * cos(P1(3))],[P1(2), P1(2) + 0.1 * sin(P1(3))],'red','LineWidth', 2);
    plot([P2(1), P2(1) + 0.1 * cos(P2(3))],[P2(2), P2(2) + 0.1 * sin(P2(3))],'green','LineWidth', 2);
    hold off
    grid on
    axis([-2, 2, -2, 2]);
end





