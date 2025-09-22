# Use official ROS Noetic base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=${DISPLAY}
ENV QT_X11_NO_MITSHM=1

# Update & install dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-opencv \
    python3-matplotlib \
    ros-noetic-tf \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy

# Set up workspace
RUN mkdir -p /root/catkin_ws/src

# Clone the repository into the container
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/Wangzhaoze/grid_mapping_noetic.git

# Install ROS dependencies
WORKDIR /root/catkin_ws
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make"

# Make script executable
RUN chmod +x /root/catkin_ws/src/grid_mapping_noetic/scripts/create_from_rosbag.py

# Source the environment
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set default command
CMD ["bash"]
