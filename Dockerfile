FROM ros:noetic-ros-core

# change shell to bash
SHELL ["/bin/bash","-c"]

# install build packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        make g++ cmake ros-noetic-tf2 ros-noetic-tf2-geometry-msgs && \
    rm -rf /var/lib/apt/lists/*

# copy vectornav repo to workspace
COPY . /catkin_ws/src/vectornav

# change to workspace
WORKDIR /catkin_ws

# build workspace
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

# create entrypoint script
RUN printf '#!/bin/bash\nsource /catkin_ws/devel/setup.bash\nexec "$@"' > /entrypoint.bash && \
    chmod 777 /entrypoint.bash

# define entrypoint
ENTRYPOINT ["/entrypoint.bash"]

# define startup default command
CMD ["roslaunch", "vectornav", "vectornav.launch"]
