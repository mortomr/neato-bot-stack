README.md
markdown
 
# 🛠️ neato-bot-stack

A portable ROS 2 Humble development stack for ARM64 robots — built and tested on Raspberry Pi 5 (Ubuntu Bookworm), using Docker, Arduino, and real-time control integration.

---

## 📦 Features

- ROS 2 Humble (desktop full) pre-installed
- Persistent workspace and `colcon` build support
- GPIO + serial enabled (RPi.GPIO, pyserial)
- Supports RViz, motor control, encoders, and custom Arduino integration
- Easy-to-launch Docker container with `docker compose`

---

## 🚀 Usage

1. Clone the repo:

```bash
git clone https://github.com/YOUR_USERNAME/neato-bot-stack.git
cd neato-bot-stack
Build the Docker container:
bash
 
docker compose build --no-cache
docker compose up -d
Enter the container:
bash
 
docker exec -it ros2_humble bash
🧱 Project Layout
graphql
 
ros2_docker/
├── Dockerfile
├── docker-compose.yml
├── ros_ws/              # ROS 2 workspace (volumed)
│   └── ros2_packages/   # Your custom ROS 2 packages
├── scripts/             # Python test utilities
├── arduino/             # Motor + encoder firmware
└── README.md
💡 Notes on ROS 2 for ARM64
Base image is ubuntu:jammy with ros-humble-desktop installed
Targeted for Raspberry Pi 5 running Ubuntu Bookworm (64-bit)
You can swap ros-humble-desktop → ros-humble-ros-base for smaller image
🧰 Development Tips
Use colcon build --packages-select your_package inside the container
ROS setup is auto-sourced in .bashrc
Logs and builds are ignored via .gitignore
📎 License
MIT — use freely, credit welcome!

