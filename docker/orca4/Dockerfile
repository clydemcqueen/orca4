FROM clydemcqueen/galactic_garden:latest
WORKDIR /home

ARG IGN_WS=/home/ign_ws
ENV APG_WS=/home/ardupilot_gazebo
ENV COLCON_WS=/home/colcon_ws

RUN apt-get update \
 && apt-get dist-upgrade -y \
 && DEBIAN_FRONTEND="noninteractive" apt-get install -y \
    bash-completion \
    keyboard-configuration \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-pip \
    python3-matplotlib \
    python3-lxml \
    python3-pygame \
    rapidjson-dev \
    software-properties-common \
 && apt-get clean

# Some handy tools
RUN apt-get update \
 && apt-get dist-upgrade -y \
 && DEBIAN_FRONTEND="noninteractive" apt-get install -y \
    git \
    glmark2 \
    iputils-ping \
    mlocate \
    vim

RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git -b ignition-garden

# Build ardupilot_gazebo
RUN [ "/bin/bash" , "-c" , " \
  cd ${APG_WS} \
  && mkdir build \
  && cd build \
  && source ${IGN_WS}/install/setup.bash \
  && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  && make -j4" ]

# Add ardupilot_gazebo to the Gazebo plugin path
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/home/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"

# ArduPilotPlugin fdm_port_in
EXPOSE 9002/udp

# Break orca4 build into multiple stages to speed up 'docker build':
# -- work that depends on workspace.repos
# -- work that depends on package.xml files
# -- everything else

RUN mkdir -p ${COLCON_WS}/src
WORKDIR ${COLCON_WS}/src

# Get workspace.repos
COPY workspace.repos .
RUN vcs import < workspace.repos

# Run rosdep over workspace.repos
RUN rosdep update && rosdep install -y --from-paths . --ignore-src

# MAVROS depends on GeographicLib, and GeographicLib needs some datasets
RUN [ "/bin/bash" , "-c" , "\
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
  && chmod +x install_geographiclib_datasets.sh \
  && ./install_geographiclib_datasets.sh" ]

# Build workspace.repos
RUN [ "/bin/bash" , "-c" , "\
  cd ${COLCON_WS} \
  && source ${IGN_WS}/install/setup.bash \
  && source /opt/ros/galactic/setup.bash \
  && colcon build" ]

COPY orca_base/package.xml orca4/orca_base/
COPY orca_bringup/package.xml orca4/orca_bringup/
COPY orca_description/package.xml orca4/orca_description/
COPY orca_msgs/package.xml orca4/orca_msgs/
COPY orca_nav2/package.xml orca4/orca_nav2/
COPY orca_shared/package.xml orca4/orca_shared/

# Run rosdep over orca4 package.xml files
RUN rosdep install -y --from-paths . --ignore-src

COPY . orca4

# Build orca4
RUN [ "/bin/bash" , "-c" , "\
  cd ${COLCON_WS} \
  && source ${IGN_WS}/install/setup.bash \
  && source /opt/ros/galactic/setup.bash \
  && colcon build" ]

ENTRYPOINT [ "/home/colcon_ws/src/orca4/entrypoint.sh" ]
