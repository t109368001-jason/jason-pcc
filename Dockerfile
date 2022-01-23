# syntax=docker/dockerfile:1
FROM ubuntu:20.04 as jpcc-u20

# disable interactive dialog
ARG TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive

# requirements
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    wget \
    build-essential \
    valgrind \
    libpcap-dev \
    libtbb-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN wget -qO- "https://cmake.org/files/v3.17/cmake-3.17.5-Linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /usr/local

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

RUN bash scripts/build.sh

###############################################################################
FROM jpcc-u20-src as jpcc-u20-build-debug

WORKDIR /jason-pcc

RUN bash scripts/build.sh