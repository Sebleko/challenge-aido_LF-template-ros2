# Definition of Submission container


ARG ARCH=arm32v7
ARG DISTRO=daffy
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG BASE_IMAGE=dt-commons

ARG DOCKER_REGISTRY=docker.io
FROM ${DOCKER_REGISTRY}/duckietown/${BASE_IMAGE}:${BASE_TAG} as BASE
WORKDIR /code


# here, we install the requirements, some requirements come by default
# you can add more if you need to in requirements.txt

ENV DEBIAN_FRONTEND=noninteractive

# DO NOT MODIFY: your submission won't run if you do
RUN apt-get update -y && \
    apt-get install -y apt-utils && \
    apt-get install -y --no-install-recommends \
         gcc \
         libc-dev\
         git \
         bzip2 \
         python3-tk \
         python3-wheel \
         python3-pip  \
         libcairo2-dev \
         libjpeg-dev\
          libgif-dev\
         software-properties-common && \
     rm -rf /var/lib/apt-get/lists/*


# RUN apt-get update -y && \
#   add-apt-repository ppa:deadsnakes/ppa -y && \
#   apt-get update -y && \
#   apt-get install -y python3.7-dev && \
#   ln -sf /usr/bin/python3.7 /usr/bin/python3


RUN mkdir -p /data/config
# TODO this is just for the default.yamls - these should really be taken from init_sd_card
RUN git clone https://github.com/duckietown/duckiefleet.git /data/config

ARG PIP_INDEX_URL="https://pypi.org/simple"

RUN echo we have PIP_INDEX_URL=${PIP_INDEX_URL} $PIP_INDEX_URL $DOCKER_REGISTRY


ENV PIP_INDEX_URL=${PIP_INDEX_URL}

RUN echo we have PIP_INDEX_URL=${PIP_INDEX_URL} $PIP_INDEX_URL
RUN env


#RUN python3 -m pip check # XXX: fails
RUN python3 -m pip list

COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN python3 -m pip install --no-cache-dir -r .requirements.txt
RUN python3 -m pip check
RUN python3 -m pip list


### Install ROS2 Foxy
ENV ROS_DISTRO=foxy
# Set locale
RUN apt-get update && apt-get install locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && export LANG=en_US.UTF-8

# Add ROS 2 apt-get repositories to system.
RUN apt-get install software-properties-common \
  && add-apt-repository universe

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
RUN apt-get upgrade -y \
  && apt-get install -y ros-foxy-base
### Install ROS2 Foxy done.

RUN mkdir launchers
COPY launchers launchers/
RUN pwd

RUN mkdir submission_underlay_ws
COPY submission_underlay_ws/src submission_underlay_ws/src

RUN mkdir dev_ws
COPY dev_ws/src dev_ws/src

COPY launchers ./

# FIXME: what is this for? envs are not persisted
RUN /bin/bash -c "export PYTHONPATH="/usr/local/lib/python3.7/dist-packages:$PYTHONPATH""

#ENV HOSTNAME=agent
ENV VEHICLE_NAME=agent
#ENV ROS_MASTER_URI=http://localhost:11311

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    ( cd submission_underlay_ws && colcon build ) && \
    . submission_underlay_ws/install/setup.sh && \
    ( cd dev_ws && colcon build )

ENV DISABLE_CONTRACTS=1
CMD ["bash", "launchers/run_and_start.sh"]
