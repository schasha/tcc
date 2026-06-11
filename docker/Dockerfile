FROM osrf/ros:humble-desktop-full

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=ros

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-humble-turtlebot3 \
        ros-humble-turtlebot3-msgs \
        ros-humble-turtlebot3-simulations \
        ros-humble-turtlebot3-gazebo \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-tf2-tools \
        ros-humble-rqt-tf-tree \
        ros-humble-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
        python3-rosdep \
        sudo vim less gdb \
    && rm -rf /var/lib/apt/lists/*

# Match host UID/GID so files edited from inside the container stay owned by the host user.
RUN if getent group ${USER_GID} >/dev/null; then \
        existing_group=$(getent group ${USER_GID} | cut -d: -f1); \
        groupmod -n ${USERNAME} "$existing_group"; \
    else \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi \
    && if getent passwd ${USER_UID} >/dev/null; then \
        existing_user=$(getent passwd ${USER_UID} | cut -d: -f1); \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m "$existing_user" || true; \
    else \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Pre-create workspace directories with correct ownership so that named volumes
# mounted on top of build/install/log inherit the right uid/gid on first run.
RUN mkdir -p /ros2_ws/src /ros2_ws/build /ros2_ws/install /ros2_ws/log \
    && chown -R ${USER_UID}:${USER_GID} /ros2_ws

USER ${USERNAME}
WORKDIR /ros2_ws

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'export TURTLEBOT3_MODEL=burger' >> /home/${USERNAME}/.bashrc \
    && echo 'export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH:-}' >> /home/${USERNAME}/.bashrc

COPY --chown=${USER_UID}:${USER_GID} entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
