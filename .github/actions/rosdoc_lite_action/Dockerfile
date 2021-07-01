FROM ros:noetic
COPY entrypoint.sh /entrypoint.sh

RUN apt-get update && apt-get install -y \
    ros-noetic-rosdoc-lite \
    graphviz

ENTRYPOINT ["/entrypoint.sh"]
