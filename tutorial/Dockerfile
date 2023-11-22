# Use Ubuntu Focal and ROS Foxy
FROM ros:foxy

# Install packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy source code
WORKDIR /ros
ENV ROS2_WS /ros
COPY src src
COPY requirements.txt requirements.txt

# Install Python packages
RUN pip install -r requirements.txt
RUN rm requirements.txt

# Build ROS packages
RUN cd ${ROS2_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
COPY ./entrypoint.sh / 
RUN chmod +x /entrypoint.sh
ENV ROS_DOMAIN_ID 10

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

