# EE3033 — Autonomous Mapping, Navigation and Target Localization

## 1. Project Overview

This project implements an autonomous object-search system using a ROS 2-based balancing robot. The system integrates LiDAR-based SLAM mapping, AMCL localization, autonomous waypoint navigation, YOLOv8 object detection, target localization and return-home navigation.

The project consists of two separate stages:

1. **Mapping Stage:** The robot is manually controlled to explore the environment while SLAM generates a 2D occupancy grid map. The completed map is then saved for autonomous navigation.
2. **Autonomous Mission:** The robot uses the saved map to generate and navigate through waypoints while searching for a target bottle. Once the bottle is detected and its approximate map coordinates are estimated, the robot stops searching and returns to its starting position.

The mapping and autonomous navigation processes are implemented separately, as they are executed at different stages of the project.

## 2. Technologies Used

- ROS 2 Humble
- SLAM Toolbox
- Nav2 Navigation Stack
- Adaptive Monte Carlo Localization (AMCL)
- YOLOv8 Object Detection
- LiDAR and USB Camera
- Python and Bash
- Raspberry Pi-based Balancing Robot
- Ubuntu Virtual Machine (VMware)
- Docker

## 3. Project Files

| File | Description |
|---|---|
| `mapping_manager.py` | Launches the SLAM mapping process, checks LiDAR and map readiness, and saves the generated occupancy grid map. |
| `auto_waypoint_object.py` | Generates and validates waypoints, performs structured navigation and rotational scanning, and manages the return-home process. |
| `object_detector.py` | Processes detections from the lab-provided YOLOv8 system, estimates the bottle's distance and map coordinates, and publishes detection confirmation signals. |
| `object_search_launch.sh` | Launches the integrated autonomous mission, including Nav2, camera, image compression/decompression, YOLOv8, object detection and waypoint navigation. |

## 4. System Workflow

### Stage 1 — Mapping

1. Launch the SLAM mapping process using `mapping_manager.py`.
2. Manually control the robot using the mobile application to explore the environment.
3. Collect LiDAR measurements to construct a 2D occupancy grid map.
4. Monitor the mapping progress in RViz2.
5. Save the completed map as `.yaml` and `.pgm` files.

### Stage 2 — Autonomous Navigation and Object Search

1. Launch the autonomous mission using `object_search_launch.sh`.
2. Load the previously generated map and initialize the robot's pose using AMCL.
3. Generate candidate waypoints within the reachable free-space regions.
4. Filter unsafe waypoints based on obstacle clearance and validate them using Nav2 path planning.
5. Arrange validated waypoints into a structured sweeping route.
6. Navigate autonomously through the waypoints and perform 360° rotational scans at selected locations.
7. Process camera images using the lab-provided YOLOv8 model to detect bottles.
8. Estimate the detected bottle's distance using its bounding-box height and calculate its approximate map coordinates using the robot's AMCL pose.
9. Confirm the bottle detection after three qualifying detections.
10. Cancel the ongoing search and autonomously return to the starting position.

If no bottle is detected after completing the waypoints, the system starts another search round, prioritizing unvisited positions before revisiting previously searched areas.

## 5. File Locations and Setup Requirements

The mapping and autonomous navigation scripts are executed separately and assume specific file locations.

### Robot — ROS 2 Docker Container

The following files should be placed inside the robot's `ros2_humble` Docker container:

| File | Expected Location |
|---|---|
| `mapping_manager.py` | `/home/ros2_ws/mapping_manager.py` |
| `auto_waypoint_object.py` | `/home/ros2_ws/auto_waypoint_object.py` |

The mapping manager saves the generated map to:

```text
/home/ros2_ws/install/balance_mapping/share/balance_mapping/maps/balance_map_current.yaml
/home/ros2_ws/install/balance_mapping/share/balance_mapping/maps/balance_map_current.pgm
```

### Ubuntu Virtual Machine

The following files should be placed in the Ubuntu VM:

| File | Expected Location |
|---|---|
| `object_search_launch.sh` | `~/object_detection/object_search_launch.sh` |
| `object_detector.py` | `~/object_detection/object_detector.py` |

The autonomous mission launch script starts the navigation, camera and image compression components inside the robot's Docker container, while image decompression, YOLOv8 and the custom object detector run on the Ubuntu VM.

### Additional Requirements

- The robot must be accessible through SSH at `balance@balance12.local`.
- The robot's Docker container must be named `ros2_humble`.
- ROS 2 Humble and the required lab-provided packages must be installed.
- The Ubuntu VM must contain the built ROS 2 workspace at `~/ros2_ws/install`.
- Both environments must use `ROS_DOMAIN_ID=12` and `ROS_LOCALHOST_ONLY=0`.
- The lab-provided `balance_bringup`, `balance_mapping` and `yolo_bringup` packages must be available.
- The camera must be accessible at `/dev/video0` inside the robot's Docker container.
- The navigation configuration must load the map generated during the mapping stage.

These paths and configurations are specific to the current project setup and may need to be modified when running the system on another device.

## 6. Running the System

### Stage 1 — Mapping

On the Ubuntu VM, connect to the robot:

```bash
ssh balance@balance12.local
```

Start the ROS 2 Docker environment:

```bash
ros2-start
```

Inside Docker, source the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=12
export ROS_LOCALHOST_ONLY=0
```

Run the mapping manager:

```bash
python3 /home/ros2_ws/mapping_manager.py
```

Manually control the robot using the mobile application while monitoring the generated map in RViz2.

Once mapping is complete, press Enter in the mapping manager terminal to save the map.

The mapping process will then stop.

### Stage 2 — Autonomous Mission

Ensure the robot's ROS 2 Docker container is running.

On the Ubuntu VM, run:

```bash
bash ~/object_detection/object_search_launch.sh
```

The launch script starts the following components:

1. Nav2 navigation and AMCL localization
2. Camera node
3. JPEG image compression
4. JPEG image decompression
5. Lab-provided YOLOv8 object detection
6. Custom bottle detection and localization

Before starting autonomous movement:

1. Open RViz2 and verify that the correct saved map is loaded.
2. Set the robot's initial position using **2D Pose Estimate**.
3. Verify that the camera, YOLOv8 and Nav2 components are functioning correctly.
4. Ensure the robot's surroundings are clear and a physical emergency stop is accessible.

Press Enter at the final launch-script prompt to start the automatic waypoint generator and autonomous navigation.

The robot will then navigate through the generated waypoints, search for the bottle, estimate its map coordinates and return to the starting position after successful detection.

## 7. Important Notes

- The mapping and autonomous mission stages are executed separately.
- The system uses the saved SLAM map for autonomous navigation.
- The waypoint generator uses Nav2 to validate candidate paths before navigation.
- The object detector relies on camera calibration parameters and an assumed bottle height for distance estimation.
- The reported bottle coordinates are approximate and may be affected by camera calibration, object dimensions and localization accuracy.
- The starting position is recorded from the first AMCL pose received by the waypoint generator.
- Autonomous navigation is enabled in the current waypoint script. The robot should only be operated after verifying localization, navigation readiness and obstacle clearance.
- The system depends on lab-provided ROS 2 packages and configurations that are not included in this repository.
