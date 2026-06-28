FROM ros:jazzy

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install ROS2 packages and tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ros2_control stack
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    ros-jazzy-teleop-twist-keyboard \
    # MuJoCo ros2_control integration
    ros-jazzy-mujoco-ros2-control \
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    # GUI/rendering support
    libglfw3-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    mesa-utils \
    # Dev tools
    git \
    vim \
    wget \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Install MuJoCo Python bindings for standalone testing
RUN pip install --break-system-packages mujoco

# Source ROS2 automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo '[ -f /workspaces/mujoco_rosmaster_m3pro/install/setup.bash ] && source /workspaces/mujoco_rosmaster_m3pro/install/setup.bash' >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /workspaces/mujoco_rosmaster_m3pro
