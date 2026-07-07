FROM python:3.11-slim-bookworm

ENV MUJOCO_GL=osmesa
ENV PYTHONUNBUFFERED=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libgl1 \
        libglib2.0-0 \
        libgomp1 \
        libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/unitree-g1-mujoco-requirements.txt
RUN pip install --no-cache-dir -r /tmp/unitree-g1-mujoco-requirements.txt

COPY python/ /opt/unitree-g1-mujoco-protocol/

WORKDIR /opt/unitree-g1-mujoco-protocol
ENTRYPOINT ["python3", "/opt/unitree-g1-mujoco-protocol/g1_protocol_sim.py"]
EXPOSE 38383 8788
