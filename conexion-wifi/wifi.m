clear all
close all

esp32IP = '10.42.0.10';
port = 80;
t = tcpclient(esp32IP, port);

node = ros2node('/imu_test');
imuSub = ros2subscriber(node, '/imu', 'sensor_msgs/Imu');

while true
    msg = receive(imuSub);
    orientation = msg.orientation;
    orientationX = orientation.x;
    disp(orientationX);

    writeline(t,num2str(orientationX));
    pause(2);
end