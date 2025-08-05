#!/bin/bash
# Set container name
CONTAINER_NAME="ros2_moveit_container"

# Build the Docker container if it doesn't exist yet
docker buildx build --platform linux/arm64 -t ros2-moveit:v1.2 --load .

# Allow Docker to use the X server for graphical applications (RViz)
xhost +local:docker

# Run the container with the necessary settings for graphical interfaces and assign a name
docker run -it --rm --network host -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI=///root/cyclonedds_config/CycloneDDS.xml -v $(pwd)/cyclonedds_config:/root/cyclonedds_config --volume=/etc/localtime:/etc/localtime:ro --volume=/etc/timezone:/etc/timezone:ro --name $CONTAINER_NAME ros2-moveit:v1.2   #--device /dev/vcan0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --privileged
