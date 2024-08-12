@echo off
set WORKSPACE=D:\Documents\ROS2_practice

docker run -it ^
    --name=ros2 ^
    -v %WORKSPACE%:/root/ROS2_practice ^
    -e DISPLAY=host.docker.internal:0.0 ^
    --net=host ^
    osrf/ros:foxy-desktop