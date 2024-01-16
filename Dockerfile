FROM ubuntu:18.04

# Fix shell configurations to avoid future errors
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Update ubuntu
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    cmake \
    curl \
    gcc \
    git \
    libglew-dev \
    libgtk2.0-dev \
    pkg-config \
    ffmpeg \
    libavformat-dev \
    libavcodec-dev \
    libswscale-dev \
    python-dev \
    python-numpy \
    unzip \
    wget \
    libeigen3-dev \
    libboost-all-dev \
    lsb-release \
    ca-certificates \
    emacs

# Set dependencies as environment variables
ENV OPENCV_VERSION=3.4.2
ENV OPENCV_DIR=opencv-$OPENCV_VERSION
ENV BOOST_VERSION=1.65.1
ENV EIGEN_VERSION=3.3.4

# Install OpenCV
WORKDIR /home
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
    && unzip opencv.zip \
    && cd $OPENCV_DIR \
    && mkdir release \
    && cd release \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local .. \
    && make \
    && make install

# Install Pangolin v0.5. Any later version leads to linking issues.
WORKDIR /home
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.5 \
    && cd Pangolin \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make

# Set up simlinks to Eigen3
RUN ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen

# Build Edge-SLAM
WORKDIR /home
RUN git clone -b docker --single-branch https://github.com/zainasir/edgeslam.git \
    && cd edgeslam \
    && chmod +x build.sh \
    && ./build.sh

# Install ROS Melodic
WORKDIR ~/
RUN ln -snf /usr/share/zoneinfo/$CONTAINER_TIMEZONE /etc/localtime && echo $CONTAINER_TIMEZONE > /etc/timezone
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install -y ros-melodic-desktop-full
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc
RUN apt install python-rosdep
RUN rosdep init
RUN rosdep update

# Download TUM RGB-D Dataset for testing
WORKDIR /home
RUN cd edgeslam \
    && mkdir Datasets \
    && cd Datasets \
    && wget https://cvg.cit.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.bag

# Download additional scripts for testing
RUN cd /home/edgeslam \
    && mkdir Scripts \
    && cd Scripts \
    && wget https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools/associate.py

RUN echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/edgeslam/Examples/ROS" >> ~/.bashrc

# Entrypoint into edgeslam
WORKDIR /home/edgeslam
