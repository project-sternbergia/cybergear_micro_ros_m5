# Cybergear ROS2 Controller

## How to Use

```bash
cd .pio/xxxx/micro_ros_arduino
docker pull microros/micro_ros_static_library_builder:humble
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble -p esp32
```

## References
