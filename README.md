# Robot CAN RPI Module

This repository sets up a ROS2-based control system for a robot using CANopen communication on a Raspberry Pi. The system is split between a Docker container (running ROS2 and robot control logic) and the host machine (handling CAN communication). Joint states are transmitted over the CANopen bus and can be visualized from a separate device on the same network.

---

## Prerequisites

Ensure the following are installed on your Raspberry Pi:
- Docker
- CAN utilities and libraries (e.g., `can-utils`)
- Proper permissions for CAN interfaces (e.g., `can0`)

---

## Setup Instructions

### 1. Build the Docker Container

Run the build script:

```bash
./build_docker.sh
```

If the script cannot be executed, make it executable first:

```bash
chmod +x build_docker.sh
```
The container only needs to be built once.

---

### 2. Configure DDS for Multi-Device Communication

Since the visualization tool is running on another device, both devices must be on the same WiFi network. Update the peer IP addresses in the following configuration file to match your setup:

**File:** `cyclonedds_config/CycloneDDS.xml`

Example snippet:

```xml
<CycloneDDS>
  <Domain id="0">
    <General>
      <DontRoute>true</DontRoute>
      <AllowMulticast>false</AllowMulticast>
      <EnableMulticastLoopback>true</EnableMulticastLoopback>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="192.168.4.159"/>
        <Peer Address="192.168.4.152"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

---

### 3. Start CAN Communication

Open a new terminal and navigate to the `can_setup` directory. Then run:

```bash
./can.sh
```

This will provide real-time feedback on the CAN communication status, such as:
- Joint states being published
- Data acquisition
- Bus availability or congestion

---

### 4. Start the Docker Container

Once CAN is up and running, launch the container:

```bash
./start_docker.sh
```

Your prompt should change to:

```
robot@...
```

---

### 5. Build and Source the ROS2 Workspace

Inside the container:

```bash
colcon build
source install/setup.bash
```

---

### 6. Launch the Robot Joint State Publisher

Start the ROS2 nodes and MoveIt configuration by running:

```bash
ros2 launch robot_moveit_config planning_execution.launch.py
```

This will:
- Publish joint states over the CAN bus
- Launch planning and execution nodes
- Connect via DDS to a remote visualization interface

To move and plan robot motion, use the remote visualization interface on the second device (refer to that repository for instructions).

---

## Troubleshooting

If any script fails to execute, make it executable:

```bash
chmod +x script_name.sh
```

Replace `script_name.sh` with the name of the script (e.g., `can.sh`, `start_docker.sh`).

---

## Repository Structure

```
rpi_ros_can_module/
├── can_setup/                   # Scripts and config for CAN interface
├── config/                      # General configuration files
├── cyclonedds_config/           # DDS config (CycloneDDS.xml)
├── robot_ws/                    # ROS2 workspace for the robot
├── ws_moveit2/                  # Optional MoveIt workspace
├── build_docker.sh              # Docker build script
├── Dockerfile                   # Docker container definition
├── start_docker.sh              # Docker start script
└── README.md                    # This file
```
