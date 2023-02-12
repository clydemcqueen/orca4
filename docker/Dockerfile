FROM osrf/ros:humble-desktop AS base

ARG USERNAME=orca4
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

# Install a few handy tools
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    bash-completion \
    build-essential \
    git \
    glmark2 \
    gnupg \
    iputils-ping \
    lsb-release \
    mlocate \
    software-properties-common \
    sudo \
    wget \
    vim \
  && rm -rf /var/lib/apt/lists/*

# Install Gazebo Garden
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    gz-garden \
  && rm -rf /var/lib/apt/lists/*

# Install NVIDIA software
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/* \
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Install some ardupilot and ardupilot_gazebo prereqs
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    python3-wxgtk4.0 \
    rapidjson-dev \
    xterm \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user
# Required for ArduSub install, but generally a good idea
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "\n# Added by orca4 Dockerfile:" >> /home/$USERNAME/.bashrc \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to our new user
USER $USERNAME
ENV USER=$USERNAME

# Clone ArduSub code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules

# Install ArduSub prereqs (this also appends to .bashrc)
WORKDIR /home/$USERNAME/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
# Note: waf will capture all of the environment variables in ardupilot/.lock-waf_linux_build.
# Any change to enviroment variables will cause a re-build.
# To avoid this call sim_vehicle.py with the "--no-rebuild" option.
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub

# Clone ardupilot_gazebo code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git

# Build ardupilot_gazebo
RUN [ "/bin/bash" , "-c" , " \
  cd ardupilot_gazebo \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  && make -j4" ]

# Break orca4 build into multiple stages to speed up 'docker build':
# -- work that depends on workspace.repos
# -- work that depends on orca4 package.xml files
# -- everything else

# Create colcon workspace and copy workspace.repos file
WORKDIR /home/$USERNAME
RUN mkdir -p colcon_ws/src/orca4
COPY --chown=$USER_UID:$USER_GID workspace.repos colcon_ws/src/orca4

# Get workspace repos
WORKDIR /home/$USERNAME/colcon_ws/src
RUN vcs import < orca4/workspace.repos

# Build for Gazebo Garden
ENV GZ_VERSION=garden

# Run rosdep over workspace repos
# Note: ros_gz asks for some Gazebo Garden keys which are not yet present in rosdistro.
# This is fine, since Gazebo Garden has already been installed.
WORKDIR /home/$USERNAME/colcon_ws
RUN rosdep update \
  && rosdep install -y --from-paths . --ignore-src --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9"

# Build everything so far
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build" ]

# Copy orca4 package.xml files
COPY --chown=$USER_UID:$USER_GID orca_base/package.xml src/orca4/orca_base/
COPY --chown=$USER_UID:$USER_GID orca_bringup/package.xml src/orca4/orca_bringup/
COPY --chown=$USER_UID:$USER_GID orca_description/package.xml src/orca4/orca_description/
COPY --chown=$USER_UID:$USER_GID orca_msgs/package.xml src/orca4/orca_msgs/
COPY --chown=$USER_UID:$USER_GID orca_nav2/package.xml src/orca4/orca_nav2/
COPY --chown=$USER_UID:$USER_GID orca_shared/package.xml src/orca4/orca_shared/

# Run rosdep over orca4 package.xml files
RUN rosdep update \
  && rosdep install -y --from-paths . --ignore-src --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9"

# MAVROS depends on GeographicLib, and GeographicLib needs some datasets
RUN [ "/bin/bash" , "-c" , "\
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
  && chmod +x install_geographiclib_datasets.sh \
  && sudo ./install_geographiclib_datasets.sh" ]

# Copy everything
COPY --chown=$USER_UID:$USER_GID . src/orca4

# Build everything
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build" ]

# Set up the environment
WORKDIR /home/$USERNAME/colcon_ws
RUN echo "export PATH=/home/$USERNAME/.local/bin:\$PATH" >> /home/$USERNAME/.bashrc \
  && echo "export PATH=/home/$USERNAME/.local/lib/python3.10/site-packages:\$PATH" >> /home/$USERNAME/.bashrc \
  && echo "source /home/$USERNAME/colcon_ws/src/orca4/setup.bash" >> /home/$USERNAME/.bashrc

# Required to use the --console option on sim_vehicle.py:
# RUN pip3 install matplotlib
