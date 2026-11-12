#!/usr/bin/env bash
set -eo pipefail

ROBOT="balance@balance12.local"
CONTAINER="ros2_humble"
SOCKET="$HOME/.ssh/object_search_socket"

export ROS_DOMAIN_ID=12
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

# This script does not stop remote Docker nodes when terminals close.
# Ensure the original bottle-search system is fully stopped before starting.
echo "Make sure the bottle-search version and old Nav2/camera nodes are stopped."
read -r -p "Press ENTER only after checking no duplicates are running..."

echo "Connecting to robot (SSH password may be requested)..."
ssh -M -S "$SOCKET" -fN -o ControlMaster=yes -o ControlPersist=yes "$ROBOT"

remote_terminal() {
    local title="$1"
    local command="$2"
    local remote_cmd="docker exec $CONTAINER bash -lc 'source /opt/ros/humble/setup.bash && source /home/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0 && exec $command'"
    gnome-terminal --title="$title" -- bash -c "ssh -S '$SOCKET' -T '$ROBOT' \"$remote_cmd\"; exec bash"
}

echo "Starting Nav2..."
remote_terminal "Navigation - Object Search" "ros2 launch balance_bringup balance_known_map_waypoint_bringup.launch.py start_waypoints:=false"
sleep 8

echo "Starting camera..."
remote_terminal "Camera - Object Search" "ros2 run v4l2_camera v4l2_camera_node --ros-args -r image_raw:=/camera/image_raw -p video_device:=/dev/video0 -p image_size:=[640,480]"
sleep 2

echo "Starting JPEG compression..."
remote_terminal "Compression - Object Search" "ros2 run image_transport republish raw compressed --ros-args --remap in:=/camera/image_raw --remap /out/compressed:=/camera/image_raw/compressed"
sleep 2

echo "Starting JPEG decompression on VM..."
gnome-terminal --title="Decompression - Object Search" -- bash -c 'source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"; export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0; ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_decompressed; exec bash'
sleep 3

echo "Starting lab YOLOv8..."
gnome-terminal --title="Lab YOLOv8" -- bash -c 'source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"; export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0; ros2 launch yolo_bringup yolov8.launch.py model:=yolov8n.pt device:=cpu input_image_topic:=/camera/image_decompressed use_tracking:=False use_3d:=False use_debug:=True; exec bash'
sleep 5

echo "Starting bottle localization from lab YOLO detections..."
gnome-terminal --title="Object Detection" -- bash -c 'source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"; export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0; python3 "$HOME/object_detection/object_detector.py"; exec bash'

echo "Open RViz2 and set 2D Pose Estimate; verify map, scan and Nav2 readiness."
echo "Keep a physical emergency stop accessible."
read -r -p "Press ENTER only when ready for autonomous movement..."

echo "Starting object waypoint generator..."
remote_terminal "Object Waypoint Generator" "python3 /home/ros2_ws/auto_waypoint_object.py"
echo "Started. Closing these windows may NOT stop processes in Docker."
