FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Set locale
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


# Install GStreamer and related plugins
RUN apt-get update && apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-dev \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
 && rm -rf /var/lib/apt/lists/*

# ROS 2 + MAVROS + Camera tools + Python + colcon + rosdep
RUN apt-get update && apt-get install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-usb-cam \
    ros-jazzy-image-tools \
    ros-jazzy-image-view \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-virtualenv \
 && rm -rf /var/lib/apt/lists/*

 # Build tools + compilers
RUN apt-get update && apt-get install -y \
    build-essential \
    pkg-config \
    cmake \
    gcc-12 \
    g++-12 \
    git \
    unzip \
    rsync \
    sudo \
    iproute2 \
    net-tools \
    udev \
    dbus \
    usbutils \
    i2c-tools \
    can-utils \
    v4l-utils \
    libc-bin \
    bsdextrautils \
    jq \
    nano \
    ffmpeg \
    libopencv-dev \
    python3-opencv \
    libzmq3-dev \
    libcairo2-dev \
    libgirepository1.0-dev \
    python-gi-dev \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gtk-3.0 \
    x11-utils \
    yq \
 && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------
# MAVROS geographic datasets
RUN  rosdep init || true && rosdep update && \
      /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# -------------------------------------------------------
# HailoRT Install
COPY hailort_4.20.0_arm64.deb /tmp/hailort.deb
RUN mkdir -p /tmp/hailort_extracted && \
      dpkg-deb -x /tmp/hailort.deb /tmp/hailort_extracted && \
      cp -a /tmp/hailort_extracted/usr/* /usr/ && \
      ldconfig && \
      rm -rf /tmp/hailort_extracted && rm -f /tmp/hailort.deb

# Create symlinks only if HailoRT is installed
RUN set -eux; \
      HAILORT_REAL="$( (ls -1 /usr/lib/libhailort.so.*.*.* 2>/dev/null || true; \
                        ls -1 /lib/libhailort.so.*.*.* 2>/dev/null || true) | head -n1 )"; \
      test -n "$HAILORT_REAL"; \
      ln -sf "$HAILORT_REAL" /usr/lib/libhailort.so.4; \
      ln -sf /usr/lib/libhailort.so.4 /usr/lib/libhailort.so; \
      ldconfig

    
# -------------------------------------------------------
# TAPPAS 3.31.0 install
ARG TAPPAS_ZIP=tappas_3.31.0_linux_installer.zip
ARG TAPPAS_DIR=tappas_v3.31.0

# Put installer zip into the image
RUN mkdir -p /tmp/setup
COPY ${TAPPAS_ZIP} /tmp/setup/${TAPPAS_ZIP}

RUN set -eux; \
    unzip /tmp/setup/${TAPPAS_ZIP} -d /tmp; \
    cd /tmp/${TAPPAS_DIR}; \
    \
    # install python 3.11 (Ubuntu 24.04 likely needs deadsnakes)
    apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y python3.11 python3.11-venv python3.11-dev && \
    rm -rf /var/lib/apt/lists/*; \
    \
    # patch installer: don't force setuptools<=66
    sed -i "s/pip3 install --upgrade pip 'setuptools<=66.0.0'/pip3 install --upgrade pip setuptools/" install.sh; \
    \
    # (optional) ensure downloader uses venv python
    sed -i 's|python3 \$TAPPAS_WORKSPACE/downloader/main.py|python \$TAPPAS_WORKSPACE/downloader/main.py|g' install.sh; \
    \
    mkdir -p hailort; \
    git clone --depth 1 https://github.com/hailo-ai/hailort.git hailort/sources; \
    chmod +x install.sh; \
    ./install.sh --skip-hailort --python-version 3.11 --target-platform arm; \
    rm -rf /tmp/${TAPPAS_DIR} && rm -rf /tmp/setup
# -------------------------------------------------------

# -------------------------------------------------------
# Create / adjust non-root user
ARG USERNAME=batuhan
ARG UID=1000
ARG GID=1000

RUN set -eux; \
    EXISTING_USER="$(getent passwd "${UID}" | cut -d: -f1 || true)"; \
    EXISTING_GROUP="$(getent group  "${GID}" | cut -d: -f1 || true)"; \
    GROUP_NAME="${EXISTING_GROUP:-$USERNAME}"; \
    \
    # Ensure group exists (create only if GID not taken)
    if [ -z "${EXISTING_GROUP}" ]; then \
        groupadd -g "${GID}" "${USERNAME}"; \
        GROUP_NAME="${USERNAME}"; \
    fi; \
    \
    if [ -n "${EXISTING_USER}" ] && [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
        # Rename existing UID user (e.g., ubuntu -> batuhan)
        usermod -l "${USERNAME}" "${EXISTING_USER}"; \
        usermod -d "/home/${USERNAME}" -m "${USERNAME}"; \
        usermod -g "${GROUP_NAME}" "${USERNAME}"; \
        usermod -s /bin/bash "${USERNAME}"; \
    elif ! id -u "${USERNAME}" >/dev/null 2>&1; then \
        # Create user if it doesn't exist at all
        useradd -m -u "${UID}" -g "${GROUP_NAME}" -s /bin/bash "${USERNAME}"; \
    fi
    

# Prepare workspace folder with appropriate permissions for non-root user
RUN mkdir -p /home/${USERNAME}/ros2_ws \
 && chown -R ${UID}:${GID} /home/${USERNAME}


# Set working directory
WORKDIR /home/${USERNAME}

# Set user to non-root 
USER ${USERNAME}

# Add entrypoint.sh to .bashrc for sourcing ROS2 environment on container start
RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc 

CMD ["/bin/bash"]
