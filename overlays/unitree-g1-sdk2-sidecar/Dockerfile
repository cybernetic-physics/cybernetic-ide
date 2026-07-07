FROM python:3.11-slim-bookworm

WORKDIR /opt/cyber-unitree-g1-sdk2-sidecar

COPY entrypoint.py /opt/cyber-unitree-g1-sdk2-sidecar/entrypoint.py

ENV PYTHONUNBUFFERED=1

ENTRYPOINT ["python3", "/opt/cyber-unitree-g1-sdk2-sidecar/entrypoint.py"]
