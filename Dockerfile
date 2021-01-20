ARG AUTOWARE_VERSION=1.14.0-melodic-cuda
ARG CARLA_VERSION="0.9.10.1"

FROM autoware/autoware:$AUTOWARE_VERSION

WORKDIR /home/autoware

# Update simulation repo to latest master.
COPY --chown=autoware update_sim.patch ./Autoware
RUN patch ./Autoware/autoware.ai.repos ./Autoware/update_sim.patch
RUN cd ./Autoware \
    && vcs import src < autoware.ai.repos \
    && git --git-dir=./src/autoware/simulation/.git --work-tree=./src/autoware/simulation pull \
    && source /opt/ros/melodic/setup.bash \
    && AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# CARLA PythonAPI
RUN mkdir ./PythonAPI
ADD --chown=autoware https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/carla-0.9.10-py2.7-linux-x86_64.egg ./PythonAPI
RUN echo "export PYTHON2_EGG=$(ls /home/autoware/PythonAPI | grep py2.)" >> .bashrc \
    && echo "export PYTHONPATH=\$PYTHONPATH:~/PythonAPI/\$PYTHON2_EGG" >> .bashrc

# CARLA ROS Bridge
# There is some kind of mismatch between the ROS debian packages installed in the Autoware image and
# the latest ros-melodic-ackermann-msgs and ros-melodic-derived-objects-msgs packages. As a
# workaround we use a snapshot of the ROS apt repository to install an older version of the required
# packages. 
RUN sudo rm -f /etc/apt/sources.list.d/ros1-latest.list
RUN sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
RUN sudo sh -c 'echo "deb http://snapshots.ros.org/melodic/2020-08-07/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-snapshots.list'
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
        python-pip \
        python-wheel \
        ros-melodic-ackermann-msgs \
        ros-melodic-derived-object-msgs \
        ros-melodic-joy* \
    && sudo rm -rf /var/lib/apt/lists/*
RUN pip install simple-pid pygame networkx==2.2

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y \
        git \
        make \
        build-essential \
        clang-8 \
        lld-8 \
        g++-7 \
        cmake \
        ninja-build \
        libvulkan1 \
        libssl-dev \
        zlib1g-dev \
        libbz2-dev \
        libreadline-dev \
        libsqlite3-dev \
        wget \
        curl \
        llvm \
        libncurses5-dev \
        libncursesw5-dev \
        xz-utils \
        tk-dev \
        libffi-dev \
        libpng-dev \
        libtiff5-dev \
        libjpeg-dev \
        tzdata \
        sed \
        curl \
        unzip \
        autoconf \
        libtool \
        rsync \
        libxml2-dev \
        libsdl-dev \
        libsdl-image1.2-dev \
        libsdl-mixer1.2-dev \
        libsdl-ttf2.0-dev \
        libsmpeg-dev \
        libportmidi-dev \
        libavformat-dev \
        libswscale-dev \
        liblzma-dev && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 && \
    update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/carla-simulator/carla.git && \
    cd carla && \
    git checkout $CARLA_VERSION && \
    make LibCarla && \
    cd Build && \
    cmake \
        -G "Ninja" \
        -DCMAKE_BUILD_TYPE=Client \
        -DLIBCARLA_BUILD_RELEASE=ON \
        -DLIBCARLA_BUILD_DEBUG=OFF \
        -DLIBCARLA_BUILD_TEST=OFF \
        -DCMAKE_TOOLCHAIN_FILE=./LibStdCppToolChain.cmake \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        .. && \
    ninja && \
    ninja install && \
    cd ../.. && \
    rm -rf carla

# CARLA Autoware agent
COPY --chown=autoware . ./carla-autoware

RUN mkdir -p carla_ws/src
RUN cd carla_ws/src \
    && ln -s ../../carla-autoware/carla-autoware-agent \
    && cd .. \
    && source /opt/ros/melodic/setup.bash \
    && catkin_make

RUN echo "export CARLA_AUTOWARE_CONTENTS=~/autoware-contents" >> .bashrc \
    && echo "source ~/carla_ws/devel/setup.bash" >> .bashrc \
    && echo "source ~/Autoware/install/setup.bash" >> .bashrc

CMD ["/bin/bash"]

