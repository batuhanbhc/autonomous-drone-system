FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

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

# -------------------------------------------------------
# Build + install GStreamer 1.22.12 from source into /usr/local (Jammy ships 1.20)
# Installs: core + base/good/bad/ugly + libav
ARG GST_VER=1.22.12

RUN apt-get update && apt-get install -y \
    # build tooling
    build-essential \
    pkg-config \
    cmake \
    ninja-build \
    git \
    bison \
    flex \
    python3 \
    python3-pip \
    # gstreamer deps
    libglib2.0-dev \
    liborc-0.4-dev \
    libunwind-dev \
    libdw-dev \
    libssl-dev \
    libjpeg-dev \
    libpng-dev \
    libx264-dev \
    libx265-dev \
    libvpx-dev \
    libvorbis-dev \
    libopus-dev \
    libtheora-dev \
    libspeex-dev \
    libmp3lame-dev \
    libaom-dev \
    libass-dev \
    libdrm-dev \
    libgbm-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dev \
    libwayland-dev \
    libx11-dev \
    libxv-dev \
    libxext-dev \
    libxrender-dev \
    libpulse-dev \
    libasound2-dev \
    libbz2-dev \
    zlib1g-dev \
    libgudev-1.0-dev \
    libmount-dev \
    libgirepository1.0-dev \
    gobject-introspection \
    gettext \
    ca-certificates \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libavfilter-dev \
    libavdevice-dev \
    libswscale-dev \
    libswresample-dev \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /usr/local/bin && \
    pip3 install --no-cache-dir --upgrade "meson>=0.62"

RUN set -eux; \
    cd /tmp; \
    for m in gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav; do \
      curl -fsSL "https://gstreamer.freedesktop.org/src/${m}/${m}-${GST_VER}.tar.xz" -o "${m}.tar.xz"; \
      tar -xf "${m}.tar.xz"; \
      cd "${m}-${GST_VER}"; \
      /usr/local/bin/meson setup build --prefix=/usr/local --buildtype=release; \
      ninja -C build; \
      ninja -C build install; \
      cd /tmp; \
      rm -rf "${m}-${GST_VER}" "${m}.tar.xz"; \
    done; \
    ldconfig

# Make sure /usr/local wins at runtime for GStreamer
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}
ENV PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:/usr/local/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH}

# -------------------------------------------------------
# ROS 2 + MAVROS + Camera tools + Python + colcon + rosdep
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-usb-cam \
    ros-humble-image-tools \
    ros-humble-image-view \
    ros-humble-tf-transformations \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-dev \
    python3-setuptools \
    python3-virtualenv \
 && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------
# Build + install OpenCV 4.6.0 from source into /usr/local (Jammy ships 4.5)
ARG OPENCV_VER=4.6.0

RUN apt-get update && apt-get install -y \
    gcc-12 \
    g++-12 \
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
    libzmq3-dev \
    libcairo2-dev \
    python-gi-dev \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gtk-3.0 \
    x11-utils \
    libgtk-3-dev \
    libtbb-dev \
    libtiff-dev \
    libopenexr-dev \
    libv4l-dev \
    python3-numpy \
 && rm -rf /var/lib/apt/lists/*


RUN set -eux; \
    cd /tmp; \
    git clone --branch "${OPENCV_VER}" --depth 1 https://github.com/opencv/opencv.git; \
    mkdir -p /tmp/opencv/build; \
    cd /tmp/opencv/build; \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D WITH_GSTREAMER=ON \
          -D WITH_FFMPEG=ON \
          -D WITH_V4L=ON \
          -D WITH_QT=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_opencv_python3=ON \
          ..; \
    cmake --build . -j"$(nproc)"; \
    cmake --install .; \
    ldconfig; \
    rm -rf /tmp/opencv

# -------------------------------------------------------
# MAVROS geographic datasets
RUN rosdep init || true && rosdep update && \
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

# TAPPAS installer checks for these *APT* package names.
# Install them so the installer doesn't fail, while runtime still prefers /usr/local (GStreamer 1.22).
RUN apt-get update && apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir yq rich pymavlink

ARG TAPPAS_ZIP=tappas_3.31.0_linux_installer.zip
ARG TAPPAS_DIR=tappas_v3.31.0

RUN mkdir -p /tmp/setup
COPY ${TAPPAS_ZIP} /tmp/setup/${TAPPAS_ZIP}

RUN set -eux; \
    unzip /tmp/setup/${TAPPAS_ZIP} -d /tmp; \
    cd /tmp/${TAPPAS_DIR}; \
    mkdir -p hailort; \
    git clone --depth 1 https://github.com/hailo-ai/hailort.git hailort/sources; \
    chmod +x install.sh; \
    ./install.sh --skip-hailort --target-platform arm; \
    rm -rf /tmp/${TAPPAS_DIR} && rm -rf /tmp/setup

# -------------------------------------------------------
# Create / adjust non-root user
ARG USERNAME=batuhan
ARG UID=1000
ARG GID=1000

RUN set -eux; \
    EXISTING_USER="$(getent passwd "${UID}" | cut -d: -f1 || true)"; \
    EXISTING_GROUP="$(getent group  "${GID}" | cut -d: -f1 || true)"; \
    GROUP_NAME="${EXISTING_GROUP:-$USERNAME}"; \
    if [ -z "${EXISTING_GROUP}" ]; then \
        groupadd -g "${GID}" "${USERNAME}"; \
        GROUP_NAME="${USERNAME}"; \
    fi; \
    if [ -n "${EXISTING_USER}" ] && [ "${EXISTING_USER}" != "${USERNAME}" ]; then \
        usermod -l "${USERNAME}" "${EXISTING_USER}"; \
        usermod -d "/home/${USERNAME}" -m "${USERNAME}"; \
        usermod -g "${GROUP_NAME}" "${USERNAME}"; \
        usermod -s /bin/bash "${USERNAME}"; \
    elif ! id -u "${USERNAME}" >/dev/null 2>&1; then \
        useradd -m -u "${UID}" -g "${GROUP_NAME}" -s /bin/bash "${USERNAME}"; \
    fi

RUN mkdir -p /home/${USERNAME}/ros2_ws \
 && chown -R ${UID}:${GID} /home/${USERNAME}

WORKDIR /home/${USERNAME}
USER ${USERNAME}

RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
