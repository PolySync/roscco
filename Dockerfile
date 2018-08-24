FROM ubuntu:16.04

WORKDIR /app

# common packages
RUN apt-get update && \
    apt-get install -y \
    build-essential cmake git wget && \
    rm -rf /var/lib/apt/lists/*

# add ROS packages to apt package manager
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# install ros
RUN apt-get update && \
    apt-get install -y ros-kinetic-ros-base

# install arduino toolchain
RUN wget -nv http://arduino.cc/download.php?f=/arduino-1.8.5-linux64.tar.xz -O arduino-1.8.5.tar.xz

RUN tar -xf arduino-1.8.5.tar.xz && \
    cd arduino-1.8.5 && \
    mkdir -p /usr/share/arduino && \
    cp -R * /usr/share/arduino
