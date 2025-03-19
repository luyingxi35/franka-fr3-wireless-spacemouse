FROM ai-companion/user_layer/libfranka-ubuntu22-fr3:0.15.0-ea82b05

USER root
ARG ROS_DISTRIBUTION=humble
ENV ROS_DISTRIBUTION=${ROS_DISTRIBUTION}

RUN add-apt-repository universe &&\
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \ 
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null &&\
    # Install apt repos
    apt update && apt install -y \
    ros-${ROS_DISTRIBUTION}-ros-base \
    ros-${ROS_DISTRIBUTION}-desktop \
    ros-dev-tools \ 
    ros-${ROS_DISTRIBUTION}-hardware-interface \
    ros-${ROS_DISTRIBUTION}-generate-parameter-library \
    ros-${ROS_DISTRIBUTION}-ros2-control-test-assets \
    ros-${ROS_DISTRIBUTION}-controller-manager \
    ros-${ROS_DISTRIBUTION}-control-msgs \
    ros-${ROS_DISTRIBUTION}-xacro \
    ros-${ROS_DISTRIBUTION}-angles \
    ros-${ROS_DISTRIBUTION}-realsense2-* \
    ros-${ROS_DISTRIBUTION}-ros2-control \
    ros-${ROS_DISTRIBUTION}-realtime-tools \
    ros-${ROS_DISTRIBUTION}-control-toolbox \
    ros-${ROS_DISTRIBUTION}-moveit \
    ros-${ROS_DISTRIBUTION}-ros2-controllers \
    ros-${ROS_DISTRIBUTION}-joint-state-publisher \
    ros-${ROS_DISTRIBUTION}-joint-state-publisher-gui \
    ros-${ROS_DISTRIBUTION}-ament-cmake \
    ros-${ROS_DISTRIBUTION}-ament-cmake-clang-format \
    ros-${ROS_DISTRIBUTION}-pinocchio \
    ros-${ROS_DISTRIBUTION}-ros-gz \
    python3-colcon-common-extensions \
    python3-rosdep \ 
    python3-rosinstall \ 
    python3-rosinstall-generator \ 
    python3-wstool \ 
    python3-opencv \
    # Realsense dependencies
    libeigen3-dev \
    python3-numpy \
    python3-tqdm \
    python3-requests \
    ros-humble-diagnostic-updater \
    ros-humble-launch-pytest \
    && rosdep init \
    && rosdep update \
    && apt clean \
    && echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.bash" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.sh" >> ~/.profile \
    && echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.bash" >> /home/$NON_ROOT_USER/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.sh" >> /home/$NON_ROOT_USER/.profile

# Build franka_ros2
ARG FRANKA_ROS2_VERSION="v1.0.0"
ARG FRANKA_DESCRIPTION_VERSION="0.4.0"
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRIBUTION}/setup.bash && \ 
    mkdir -p /tmp/franka_ros2 && cd /tmp/franka_ros2 && \
    git clone --recursive https://github.com/frankaemika/franka_ros2.git --branch ${FRANKA_ROS2_VERSION} && \
    git clone --recursive https://github.com/frankaemika/franka_description.git --branch ${FRANKA_DESCRIPTION_VERSION} && \
    sed -i "s/cartesian_velocity_command_rate_limit_active_{false}/cartesian_velocity_command_rate_limit_active_{true}/" /tmp/franka_ros2/franka_ros2/franka_hardware/include/franka_hardware/robot.hpp &&\
    colcon build --install-base /opt/ros/${ROS_DISTRIBUTION}/franka --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off && \
    cd .. &&\
    rm -rf /tmp/franka_ros2 && \
    echo "source /opt/ros/${ROS_DISTRIBUTION}/franka/setup.bash" >> ~/.bashrc &&\
    echo "source /opt/ros/${ROS_DISTRIBUTION}/franka/setup.sh" >> ~/.profile &&\
    echo "source /opt/ros/${ROS_DISTRIBUTION}/franka/setup.bash" >> /home/$NON_ROOT_USER/.bashrc &&\
    echo "source /opt/ros/${ROS_DISTRIBUTION}/franka/setup.sh" >> /home/$NON_ROOT_USER/.profile'

RUN pip install pyspacemouse 

RUN apt-get update && apt-get install -y \
    libhidapi-dev \
    && rm -rf /var/lib/apt/lists/*

# TODO: Add udev rules for hidraw devices if you want to use the devcontainer remote user franka_dev instead of root (requires fixing)
# RUN echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' > /etc/udev/rules.d/99-hidraw-permissions.rules \
#     && usermod -aG plugdev $USER \
#     && newgrp plugdev

RUN pip install easyhid



