# Minimal ROS 2 Jazzy image for the rail-runner workspace.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=user

# Workspace dependencies. rosdep reads the package manifests, so this layer
# stays correct if the packages gain new dependencies.
COPY rail-interface/package.xml /tmp/ws/rail-interface/package.xml
COPY rail-bringup/package.xml /tmp/ws/rail-bringup/package.xml

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        sudo \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install --from-paths /tmp/ws --ignore-src --rosdistro $ROS_DISTRO -y \
    && rm -rf /tmp/ws \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu Noble ships a default "ubuntu" user at UID/GID 1000; drop it so the
# container user can match the host and keep the bind-mounted workspace
# writable. dialout is GID 20 in both the host and the image.
RUN if getent passwd $USER_UID > /dev/null; then \
        userdel -r "$(getent passwd $USER_UID | cut -d: -f1)"; \
    fi \
    && if ! getent group $USER_GID > /dev/null; then groupadd --gid $USER_GID $USERNAME; fi \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG dialout $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo '[ -f /ros2_ws/scripts/ros_env.sh ] && source /ros2_ws/scripts/ros_env.sh' \
        >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /ros2_ws
CMD ["bash"]
