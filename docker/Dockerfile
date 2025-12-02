FROM ros:jazzy

RUN apt-get update && apt-get install -y \
        ros-jazzy-domain-bridge \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-xacro \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-foxglove-msgs \
        ros-jazzy-tf-transformations \
        ros-jazzy-septentrio-gnss-driver \
        ros-jazzy-usb-cam \
        ros-jazzy-image-transport \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-image-tools \
        ros-jazzy-compressed-image-transport \
        ros-jazzy-axis-camera \
        python3-pip \
        python3-serial \
        python3-requests \
        python3-yaml \
        git \
        nano \
        wget \
        unzip \
        iputils-ping

# Setup lizard
RUN mkdir -p /root/.lizard && \
    cd /root && \
    LATEST_VERSION=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget https://github.com/zauberzeug/lizard/releases/download/${LATEST_VERSION}/lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip && \
    unzip lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip -d /root/.lizard && \
    chmod +x /root/.lizard/espresso.py && \
    cd /root/.lizard && \
    pip install --break-system-packages -r requirements.txt && \
    cd /root && \
    rm -f lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip && \
    echo "export PYTHONPATH=/root/.lizard:$PYTHONPATH" >> /root/.bashrc

COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/home/zauberzeug/.cache/pip \
    pip install --break-system-packages --ignore-installed -r /tmp/requirements.txt

COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc

# Copy ROS packages
COPY ./devkit_driver /workspace/src/devkit_driver
COPY ./devkit_launch /workspace/src/devkit_launch
COPY ./devkit_ui /workspace/src/devkit_ui

# Build ROS packages
WORKDIR /workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select devkit_driver devkit_launch devkit_ui --symlink-install && \
    . install/setup.sh

WORKDIR /workspace
