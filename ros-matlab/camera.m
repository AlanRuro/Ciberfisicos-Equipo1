node = ros2node("/matlab_camera_node");

% Suscribirse al tópico /image_raw
imageSub = ros2subscriber(node, "/image_raw", "sensor_msgs/Image");

while true
    % Receive the message
    [imgData, imgStatus, imgStatusText] = receive(imageSub);
    
    % Extract image data
    a = rosReadImage(imgData);
    
    % Show image
    figure(1);
    imshow(a);
    drawnow;
end