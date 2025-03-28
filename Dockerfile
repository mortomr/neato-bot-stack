# Use the official Ubuntu 22.04 (Jammy) base image
FROM ubuntu:jammy

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8

# Install tools needed to fetch and add the ROS 2 key
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales

# Add ROS 2 apt repository and install core packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y \
    python3-pip \
    nano \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-pytest-cov \
    ros-humble-ros2run \
    ros-humble-ros2launch \
    ros-humble-serial-driver \
    screen \
    tree && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean


# Install RPi.GPIO and pyserial using pip
RUN pip3 install RPi.GPIO pyserial && \
    rm -rf ~/.cache/pip


# Source the ROS 2 setup script and workspace in the root user's bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ros_ws/install/setup.bash" >> /root/.bashrc

# Set default command to bash
CMD ["/bin/bash"]
ENV ROS_WORKSPACE=/ros_ws


ARG USERNAME=mark
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo,dialout $USERNAME

USER $USERNAME
