FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# Basics and locales
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl gnupg2 lsb-release ca-certificates \
    software-properties-common build-essential git \
    python3-pip python3-venv \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS 2 apt repo
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg2 lsb-release \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
ARG ROS_META=ros-humble-desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ${ROS_META} \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

WORKDIR /root/nuwave-rov

# Copy the workspace to the docker container
COPY . /root/nuwave-rov

# --- ADD THIS BLOCK ---
# Provide the entrypoint script
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sed -i 's/\r$//' /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh
# --- END BLOCK ---

# Install deps
RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths /root/nuwave-rov/src --ignore-src -y --skip-keys="ament_python cv2" && \
    apt-get install -y --no-install-recommends ros-humble-rosbridge-server python3-opencv && \
    rm -rf /var/lib/apt/lists/*

# Install Python deps (pyfirmata, pygame, etc.) not covered by rosdep
RUN pip3 install --no-cache-dir -r /root/nuwave-rov/requirements.txt

# Build the workspace so install/ exists in the image
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Source ROS + the workspace in every interactive shell (including `docker exec`)
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /root/nuwave-rov/install/setup.bash ] && source /root/nuwave-rov/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
