FROM ros:jazzy-ros-base

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
