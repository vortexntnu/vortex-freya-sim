FROM ros:humble-perception

RUN DEBIAN_FRONTEND=noninteractive

# Default to using bash > sh
RUN ln -sf /bin/bash /bin/sh

# Install Gazebo Garden
RUN apt-get update && apt-get install -y lsb-release curl gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y gz-garden python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro

WORKDIR /ros_ws

RUN mkdir -p /ros_ws/src

COPY . /ros_ws/src

RUN source /opt/ros/humble/setup.bash && cd /ros_ws && colcon build --merge-install

ENTRYPOINT ["/bin/bash", "-c", "/ros_ws/src/vortex_sim_interface/docker_entrypoint.sh"]