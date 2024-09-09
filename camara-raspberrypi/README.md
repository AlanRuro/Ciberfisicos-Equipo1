# Inicializar cámara
## Dependencias
`sudo apt update`\
`sudo apt install v4l-utils`\
`sudo apt install ros-humble-v4l2-camera`\
`sudo apt install rviz2`

Provee los nodos necesarios para capturar imágenes a partir de una cámara compatible con v4l2

## Verificación
`v4l2-ctl --list-devices`

Verifica que la cámara es detectada.\
Debe aparecer de la siguiente manera:\
`mmal service 16.1 (platform:bcm2835-v4l2-0):
/dev/video0`

## Conexión
`ssh ubuntu@10.42.0.1`

Se conecta al robot mediante ssh con password **student** y se ejecuta dentro del robot:

`ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[320,240]" -p
framerate:=30 --ros-args --param reliability:=best_effort --param history:=keep_last
--param depth:=10`

## Probar cámara
En la laptop ejecutar:

`ros2 run rqt_image_view rqt_image_view`



