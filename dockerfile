FROM ros:jazzy-ros-base

WORKDIR /workspace

# Install deps
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys=ament_python && \
    apt-get install -y --no-install-recommends ros-jazzy-rosbridge-server && \
    rm -rf /var/lib/apt/lists/*


