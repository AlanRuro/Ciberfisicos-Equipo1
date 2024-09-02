clear all
close all
clc

x = 0;
y = 0;
theta = 0;

tf = 10;
t = 0;
dt = 0.01;

kpd = -1;
kpr = 1;

xd = 1;
yd = 1;
Thetae = [];
i = 1;

vMax = 0.18;
wMax = 0.4;


node = ros2node('/velocity');
velPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
velSub = ros2subscriber(node, 'cmd_vel', 'geometry_msgs/Twist');
velocityMsg = ros2message("geometry_msgs/Twist");

tic
while t < tf
    
    %% Cálculo de referencias
    thetad = atan2((yd-y),(xd-x));
    
    %% Cálculo de errores
    d = sqrt((x-xd)^2 + (y-yd)^2);
    thetae = theta - thetad;
    if thetae > pi
        thetae = thetae - 2*pi;
    elseif thetae < - pi
        thetae = thetae + 2*pi;
    end

    %% Condicional para reversa
    if thetae > pi/2
        thetae = thetae - pi;
        d = -d;
    elseif thetae < - pi/2
        thetae = thetae + pi;
        d = -d;
    end
    
    %% Leyes de control
    if abs(thetae) < pi/32
        V = kpd*d;
        V = saturation(V, vMax);
    else
        V = 0;
    end

    w = -kpr*thetae;
    w = saturation(w, wMax);

    % Cercania al punto
    if abs(d) < 0.2
        V = 0;
        w = 0;
    end

    %% Simulación del robot

    xp = V*cos(theta);
    yp = V*sin(theta);
    thetap = w;
    
    %% Send velocities to robot
    velocityMsg.linear.x = V;
    velocityMsg.linear.y = 0;
    velocityMsg.angular.z = w;
    send(velPub, velocityMsg);

    x = x + xp*dt;
    y = y + yp*dt;
    theta = theta + thetap*dt;

    
    dt = toc;
    tic; 
    t = t + dt;

    Thetae(i) = thetae;
    i = i + 1;

    figure(1)
    scatter(x,y,'color','blue','LineWidth',2);
    hold on
    plot([x,x+0.5*cos(theta)],[y,y+0.5*sin(theta)],'color','red','LineWidth',2)
    hold off
    grid on
    axis([-10,10,-10,10]);
end

figure(2)
plot(Thetae)

function vel = saturation(vel, velMax)
    % if vel > velMax
    %     vel = velMax;
    % elseif vel < -velMax
    %     vel = -velMax;
    % end

    vel = velMax * tanh((vel) / velMax);
end