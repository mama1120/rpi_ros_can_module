# Start from the official ROS2 Humble MoveIt2 image
FROM ros:humble-ros-base-jammy
    
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-msgs \
    ros-humble-moveit-core \
    ros-humble-graph-msgs  \
    ros-humble-rviz-visual-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit-visual-tools \
    ros-humble-example-interfaces \
    ros-humble-demo-nodes-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    iputils-ping \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt update && apt install -y python3-rosdep \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update
    
RUN apt-get update && apt-get install -y ros-humble-moveit-visual-tools

RUN apt update && apt install -y python3-pip && \
	pip install --no-cache-dir canopen
	
RUN apt-get update && apt-get install -y \
	can-utils \
	iproute2

RUN apt-get update && apt-get install -y ros-humble-diagnostic-updater || true

# Copy only the src folder from your host ROS2 workspace to the container
COPY robot_ws/src /root/ros2_ws/src/

# Copy MoveIt2 workspace into the container
COPY ws_moveit2 /root/ws_moveit2

# Set the working directory to the ROS2 workspace
WORKDIR /root/ros2_ws/

#RUN rosdep install --from-paths src/ros2_canopen --ignore-src -r --rosdistro humble -y

# Source the setup scripts of ROS2 and MoveIt2, and build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"

# Source the ROS2 setup script for the environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set environment variable for X11 display (optional, can be set when running the container)
ENV DISPLAY=:0
    
