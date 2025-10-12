FROM ubuntu:24.04

RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Install Git
RUN apt-get update && apt-get install git -y

# ensure that the Ubuntu Universe repository is enabled
RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install -y curl

# The ros-apt-source packages provide keys and apt source configuration for the various ROS repositories
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb

# Install ROS2
RUN apt update
RUN apt upgrade -y
RUN apt install -y ros-jazzy-desktop
WORKDIR /workspace

# The worskpace to the docker container
COPY . /workspace

# Install deps
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths /workspace --ignore-src -y --skip-keys=ament_python && \
    apt-get install -y --no-install-recommends ros-jazzy-rosbridge-server && \
    rm -rf /var/lib/apt/lists/*

RUN sed -i 's/\r$//' /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
