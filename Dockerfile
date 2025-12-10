FROM osrf/ros:jazzy-desktop
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    python3-pandas \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ros2_ws
COPY ./src /ros2_ws/src
COPY ./data /ros2_ws/data
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
CMD ["bash"]