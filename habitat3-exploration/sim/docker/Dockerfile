# Habitat 3.0 (habitat-sim 0.3.1 + habitat-lab v0.3.1) + ROS 2 Jazzy bridge.
#
# Base: Ubuntu 24.04 (Noble) for official ROS 2 Jazzy packages. GPU driver
# libraries are injected at runtime by the NVIDIA container toolkit.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl wget git git-lfs bzip2 tmux procps \
        xvfb x11vnc novnc websockify feh \
        libglvnd0 libgl1 libegl1 libgles2 libopengl0 libglx0 \
        libx11-6 libxext6 libsm6 libxrender1 \
        python3-pip python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy (slim — bridge only, no MAVROS/Gazebo)
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y \
      ros-dev-tools \
      ros-jazzy-ros-base \
      ros-jazzy-rosidl-default-generators \
      ros-jazzy-cv-bridge \
      ros-jazzy-tf2-ros \
      ros-jazzy-image-transport \
      ros-jazzy-map-msgs \
      ros-jazzy-ament-cmake-gtest \
      python3-colcon-common-extensions \
      python3-empy \
      libopencv-dev \
      libboost-graph-dev \
    && rm -rf /var/lib/apt/lists/*

# Miniforge (conda) — habitat-sim prebuilt packages
RUN wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh \
        -O /tmp/miniforge.sh \
    && bash /tmp/miniforge.sh -b -p /opt/conda \
    && rm /tmp/miniforge.sh
ENV PATH=/opt/conda/bin:$PATH

RUN conda create -y -n habitat python=3.9 \
    && conda install -y -n habitat habitat-sim=0.3.1 withbullet headless \
         -c conda-forge -c aihabitat \
    && conda clean -afy

RUN /opt/conda/envs/habitat/bin/pip install --no-cache-dir \
        "git+https://github.com/facebookresearch/habitat-lab.git@v0.3.1#subdirectory=habitat-lab" \
        imageio opencv-python "numpy<2"

# Python deps for ROS bridge + mission nodes (system Python 3.12 used by rclpy)
RUN pip3 install --no-cache-dir --break-system-packages numpy pillow imageio requests opencv-python

# --- EGL plumbing for headless NVIDIA rendering (incl. WSL2) -----------------
RUN mkdir -p /usr/share/glvnd/egl_vendor.d \
    && printf '{\n  "file_format_version": "1.0.0",\n  "ICD": {\n    "library_path": "libEGL_nvidia.so.0"\n  }\n}\n' \
       > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN rm -f /opt/conda/envs/habitat/lib/libEGL.so* \
          /opt/conda/envs/habitat/lib/libEGL_mesa* \
          /opt/conda/envs/habitat/lib/libGLX_mesa* \
          /opt/conda/envs/habitat/lib/libGLdispatch.so* \
          /opt/conda/envs/habitat/lib/libOpenGL.so* \
          /opt/conda/envs/habitat/lib/libGL.so* \
          /opt/conda/envs/habitat/lib/libGLX.so* || true

ENV __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    MAGNUM_LOG=quiet \
    HABITAT_SIM_LOG=quiet \
    DISPLAY=:1 \
    HABITAT_DATA_PATH=/data

WORKDIR /workspace
RUN mkdir -p /workspace/missions /workspace/scripts /data /opt/explorer_workspace

# Build context is habitat3-exploration/ project root (see docker-compose.yml)
COPY sim/scripts/ /workspace/scripts/
RUN chmod +x /workspace/scripts/*.sh

COPY sim/scripts/elytra_view_server.py /opt/elytra/elytra_view_server.py

COPY ros_workspace/src/ /opt/explorer_workspace/ros_workspace/src/

RUN /bin/bash -lc "export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin && \
    source /opt/ros/jazzy/setup.bash && \
    cd /opt/explorer_workspace/ros_workspace && \
    colcon build --symlink-install"

COPY sim/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 6080 8090
ENTRYPOINT ["/entrypoint.sh"]
