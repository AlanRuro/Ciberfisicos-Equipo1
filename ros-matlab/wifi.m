clear all
close all

esp32IP = '10.42.0.10';
port = 80;
t = tcpclient(esp32IP, port);

node = ros2node('/imu_test');
imuSub = ros2subscriber(node, '/imu', 'sensor_msgs/Imu');

while true
    msg = receive(imuSub, 10);
    orientation = msg.orientation;
    orientationX = orientation.x;
    writeline(t,string(orientationX));
    disp(orientationX);
    pause(2);
end