FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy this repo as a ROS 2 package tree inside src/
COPY . ./src/Tools-and-Software-Project

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
