FROM pytorch/pytorch:1.12.0-cuda11.3-cudnn8-runtime

ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ARG USER_ID
ARG GROUP_ID

ARG DEBIAN_FRONTEND=noninteractive

RUN pip install albumentations
RUN pip install segmentation-models-pytorch
RUN apt-get update
RUN apt-get install nano

RUN apt-get update && apt-get install -y gnupg

RUN apt-get update && apt-get install -y lsb-release && apt-get clean all

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt install curl -y

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update

RUN apt install ros-melodic-desktop-full -y
RUN apt-get install ros-melodic-ros-numpy


RUN pip install PyYAML && pip install rospkg && pip install empy && pip install opencv-python && pip install opencv-contrib-python && pip install wandb

SHELL [ "/bin/bash" , "-c" ]


