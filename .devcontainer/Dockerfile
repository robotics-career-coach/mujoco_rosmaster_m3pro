FROM ros:humble

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create non-root user (ros:humble has an 'ubuntu' user at UID/GID 1000; rename it)
RUN if id -u $USER_UID >/dev/null 2>&1; then \
        OLD_USER=$(getent passwd $USER_UID | cut -d: -f1); \
        usermod -l $USERNAME -d /home/$USERNAME -m -s /bin/bash "$OLD_USER"; \
        groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1) 2>/dev/null || true; \
    else \
        groupadd --gid $USER_GID $USERNAME; \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install ROS2 packages and tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ros2_control stack
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    # MuJoCo ros2_control integration
    ros-humble-mujoco-ros2-control \
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    # GUI/rendering support
    libglfw3-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dri \
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
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo '[ -f /workspaces/mujoco_rosmaster_m3pro/install/setup.bash ] && source /workspaces/mujoco_rosmaster_m3pro/install/setup.bash' >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /workspaces/mujoco_rosmaster_m3pro
