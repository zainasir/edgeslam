FROM ubuntu:18.04

# Copy git contents to container filesystem
WORKDIR /home/edgeslam
COPY . .

# Set dependencies as environment variables
ENV OPENCV_VERSION=3.4.2
ENV OPENCV_DOWNLOAD_URL=https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip
ENV OPENCV_DIR=opencv-$OPENCV_VERSION
ENV BOOST_VERSION=1.65.1
ENV EIGEN_VERSION=3.3.4

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
    libavcodec-dev \
    libswscale-dev \
    python-dev \
    python-numpy \
    unzip

# Install OpenCV
WORKDIR /home
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip