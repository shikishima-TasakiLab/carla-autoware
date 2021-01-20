FROM ubuntu:18.04

ARG PYTHON_VERSION="3.7.9"
ARG CARLA_VERSION="0.9.10.1"
ARG CARLA_EGG_VERSION="0.9.10"
ARG HOME="/root"
ARG PYTHON_ROOT="$HOME/local/python-$PYTHON_VERSION"
ARG PYENV_ROOT="$HOME/.pyenv"
ENV PATH="$PYTHON_ROOT/bin:$PATH"

WORKDIR /root/

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y && \
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

RUN git clone https://github.com/pyenv/pyenv.git $PYENV_ROOT && \
    $PYENV_ROOT/plugins/python-build/install.sh && \
    /usr/local/bin/python-build -v $PYTHON_VERSION $PYTHON_ROOT && \
    rm -rf $PYENV_ROOT

RUN pip install -Iv \
        setuptools==47.3.1 && \
    pip install \
        distro \
        numpy \
        pygame

RUN ln -s ${HOME}/local/python-${PYTHON_VERSION}/include/python3.7m ${HOME}/local/python-${PYTHON_VERSION}/include/python3.7

RUN git clone -b $CARLA_VERSION https://github.com/carla-simulator/carla.git && \
    cd carla && \
    make PythonAPI && \
    mkdir -p -v /opt/carla && \
    cp -r -v PythonAPI /opt/carla/ && \
    cd .. && \
    rm -rf carla
RUN ln -s /opt/carla

ENV PYTHONPATH "${PYTHONPATH}:/opt/carla/PythonAPI/carla/dist/carla-${CARLA_EGG_VERSION}-py3.7-linux-x86_64.egg"
