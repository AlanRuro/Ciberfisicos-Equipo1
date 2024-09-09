% Ros
node = ros2node('/velocity');

velPub = ros2publisher(node, '/tb3_1/cmd_vel', 'geometry_msgs/Twist');
velSub = ros2subscriber(node, '/tb3_1/cmd_vel', 'geometry_msgs/Twist');
velocityMsg = ros2message("geometry_msgs/Twist");

velocityMsg.linear.x = 0;
velocityMsg.linear.y = 0;
velocityMsg.angular.z = 0;

velPub2 = ros2publisher(node, '/tb3_0/cmd_vel', 'geometry_msgs/Twist');
velSub2 = ros2subscriber(node, '/tb3_0/cmd_vel', 'geometry_msgs/Twist');
velocityMsg2 = ros2message("geometry_msgs/Twist");

velocityMsg2.linear.x = 0;
velocityMsg2.linear.y = 0;
velocityMsg2.angular.z = 0;

send(velPub, velocityMsg);
% send(velPub2, velocityMsg2);