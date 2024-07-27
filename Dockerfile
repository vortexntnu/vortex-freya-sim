FROM ros:humble-perception

WORKDIR /ros_ws

RUN mkdir -p /ros_ws/src

COPY . /ros_ws/src

RUN source /opt/ros/humble/setup.bash && cd /ros_ws && colcon build --merge-install

ENTRYPOINT ["/bin/bash", "-c", "/ros_ws/src/vortex_sim_interface/run_sim.sh"]