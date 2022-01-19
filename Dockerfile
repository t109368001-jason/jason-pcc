FROM ubuntu:20.04 as jpcc-u20

# disable interactive dialog
ARG TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive

# requirements
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    valgrind \
    libpcap-dev \
    libtbb-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

###############################################################################
FROM jpcc-u20 as jpcc-u20-env

# requirements
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    openssh-server \
    curl \
    tar \
    wget \
    rsync \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -rm -d /home/ubuntu -s /bin/bash -g root -G sudo -u 1001 ubuntu
RUN mkdir /var/run/sshd
RUN echo 'ubuntu:ubuntu' | chpasswd
RUN echo 'root:root' | chpasswd
RUN service ssh start

RUN mkdir /home/ubuntu/.ssh
RUN echo "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQDoGyEMPMGVnD58XejnEYfPE+90/x6cRWyIZba6JY7m+eUGM/RBMK56Q+iBmitmAqVtrf5qWD5HmV88BGJtlDWyDnHac7bu/vpIqzdYJcBivEahtPqgk9EJpjA0SYVYqUU+xW4KPlndyFvOvg67W0vQPvb1y6NDmTGw//hzZnei8ycmLAADl6I0dfVwMsURbJr2gjfiCwjBenbofjhVIsB2dFstgz8PvO7ldXirE5IJvXNh+YzpTOMaQNg9fmpTR5CmMEMwnBLHv9Wr2/mcqAC2oJhtyK+S8XB/DnvfhErwqc5O9s1dCIJlaqC2bYSi/Z2sxMqTpkhQxRcjxRGA6VNUWkKJ19kWS/OOGGHE11j6KWRuqSPpKf1/0ZhOOcL1uIWSxUI6VOVE1HJ6a5YjvwbKkZQ1Ufh9NxV9fA4lMiYeFNavxOkNT2TQQUUmIuIS6WwPIrogJCTqtAYliKnXo/fsUDKqiwGuOxgUSekvNLBitQiuqeLsKUuZRAkUJSk3MJM= wesdx@XIAO-PC" > /home/ubuntu/.ssh/known_hosts

EXPOSE 22

CMD ["/usr/sbin/sshd","-D"]

###############################################################################
FROM jpcc-u20 as jpcc-u20-src

COPY ./scripts/build.sh /jason-pcc/scripts/
COPY ./scripts/test.sh /jason-pcc/scripts/
COPY ./CMakeLists.txt /jason-pcc/
COPY ./cmake /jason-pcc/cmake
COPY ./libs /jason-pcc/libs
COPY ./app /jason-pcc/app
COPY ./tests /jason-pcc/tests

RUN mkdir -p build

###############################################################################
FROM jpcc-u20-src as jpcc-u20-build-release

WORKDIR /jason-pcc

RUN scripts/build.sh

###############################################################################
FROM jpcc-u20-src as jpcc-u20-build-debug

WORKDIR /jason-pcc

RUN scripts/build.sh