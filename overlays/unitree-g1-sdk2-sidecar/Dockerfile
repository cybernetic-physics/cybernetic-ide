FROM python:3.11-slim-bookworm

WORKDIR /opt/cyber-unitree-g1-sdk2-sidecar

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        cyclonedds-dev \
        libboost-all-dev \
        libeigen3-dev \
        libfmt-dev \
        libglfw3-dev \
        libgl1 \
        libglib2.0-0 \
        libosmesa6 \
        libspdlog-dev \
        libyaml-cpp-dev \
        pkg-config \
        xauth \
        xvfb \
    && mkdir -p /opt/cyclonedds \
    && ln -s /usr/include /opt/cyclonedds/include \
    && ln -s /usr/bin /opt/cyclonedds/bin \
    && ln -s /usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH) /opt/cyclonedds/lib \
    && rm -rf /var/lib/apt/lists/*

ENV CYCLONEDDS_HOME=/opt/cyclonedds

RUN CMAKE_PREFIX_PATH=/opt/cyclonedds \
    python3 -m pip install --no-cache-dir cyclonedds==0.10.2 numpy

COPY entrypoint.py /opt/cyber-unitree-g1-sdk2-sidecar/entrypoint.py

ENV PYTHONUNBUFFERED=1

ENTRYPOINT ["python3", "/opt/cyber-unitree-g1-sdk2-sidecar/entrypoint.py"]
