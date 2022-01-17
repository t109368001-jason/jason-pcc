FROM ubuntu:20.04 as jpcc-u20-env

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

FROM jpcc-u20-env as jpcc-u20-src

COPY ./app /jason-pcc/app
COPY ./cmake /jason-pcc/cmake
COPY ./libs /jason-pcc/libs
COPY ./scripts /jason-pcc/scripts
COPY ./tests /jason-pcc/tests
COPY ./CMakeLists.txt /jason-pcc/

RUN mkdir -p build

FROM jpcc-u20-src as jpcc-u20-build-release

ENV BUILD_TYPE="Release"

WORKDIR /jason-pcc

RUN bash scripts/jpcc-build.sh

FROM jpcc-u20-src as jpcc-u20-build-debug

ENV BUILD_TYPE="Debug"

WORKDIR /jason-pcc

RUN bash scripts/jpcc-build.sh
