FROM ros:melodic
SHELL ["bash", "-c"]

# Basic Setting
ENV LANG="en_US.UTF-8"

# Copy files
RUN mkdir -p /opt/ros-video-grabber
COPY ./src /opt/ros-video-grabber/src
WORKDIR /opt/ros-video-grabber
RUN source /opt/ros/*/setup.bash \
  && cd src \
  && catkin_init_workspace \
  && cd .. \
  && apt update \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y \
  && catkin_make \
  && apt -y clean \
  && rm -rf /var/lib/apt/lists/*

COPY ./docker-entrypoint.sh /opt/ros-video-grabber/docker-entrypoint.sh
ENTRYPOINT ["/opt/ros-video-grabber/docker-entrypoint.sh"]

# Default CMD
CMD ["/bin/bash"]
