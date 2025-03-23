#!/bin/bash
# Always run from the directory where this script lives
cd "$(dirname "$0")/.."

# Start the container if it's not already running
docker compose up -d

# Wait for it to spin up (especially on first run)
sleep 2

# Attach only if it’s running
if docker ps --format '{{.Names}}' | grep -q '^ros2_humble$'; then
  docker exec -it ros2_humble bash
else
  echo "❌ Container 'ros2_humble' not running or not found."
  docker ps -a
fi
