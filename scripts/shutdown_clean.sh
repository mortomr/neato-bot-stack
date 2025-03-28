#!/bin/bash
echo "Stopping ROS container..."
docker stop ros2_humble
echo "Shutting down system..."
sudo shutdown now
