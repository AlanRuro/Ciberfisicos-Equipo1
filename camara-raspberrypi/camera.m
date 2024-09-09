node = ros2node("/camara");
imageSub = ros2subscriber(node, "/image_raw", "sensor_msgs/Image");

while true
    imgData = receive(imageSub);
    
    img = rosReadImage(imgData);
    
    figure(1);
    imshow(img);
    drawnow;
end