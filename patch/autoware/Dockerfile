FROM llccc/autoware-v1-determinism:latest

USER root

# The upstream image was built in 2024 and contains the ROS archive key with
# its old expiry date. Refresh the same official key so Rocker can securely
# install its runtime helpers.
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && chmod 0644 /usr/share/keyrings/ros-archive-keyring.gpg \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*
