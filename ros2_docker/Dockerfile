FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Locale
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
 http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
 > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    sudo \
    ros-humble-desktop \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-usb-cam \
    ros-humble-image-tools \
    ros-humble-image-view \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    iproute2 \
    net-tools \
    udev \
    dbus \
    usbutils \
    i2c-tools \
    can-utils \
    v4l-utils \
    build-essential \
    nano \
    jq \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*
    
    
RUN pip3 install --no-cache-dir yq

# MAVROS geographic datasets
RUN rosdep init || true && rosdep update
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Create non-root user matching host UID/GID
ARG USERNAME=batuhan
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && usermod -aG dialout,video,audio,plugdev,sudo ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# Prepare workspace
RUN mkdir -p /home/${USERNAME}/ros2_ws \
 && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

WORKDIR /home/${USERNAME}

RUN echo "source /entrypoint.sh" >> /home/batuhan/.bashrc 

CMD ["/bin/bash"]
