# syntax=docker/dockerfile:1
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