# syntax=docker/dockerfile:1
FROM ubuntu:20.04 as jpcc-u20

# disable interactive dialog
ARG TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive

# requirements
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    libpcap-dev \
    libtbb-dev \
    libpcl-dev \
    valgrind \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN wget -qO- "https://cmake.org/files/v3.17/cmake-3.17.5-Linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /usr/local

###############################################################################
FROM jpcc-u20 as jpcc-u20-src

WORKDIR /

RUN git clone --depth 1 --branch main https://${JASON_PCC_GITHUB_TOKEN}github.com/t109368001-jason/jason-pcc.git

WORKDIR /jason-pcc

###############################################################################
FROM jpcc-u20-src as jpcc-u20-build-release

RUN mkdir -p build

RUN bash scripts/build-release.sh

ADD ./scripts/*.sh /jason-pcc/

###############################################################################
FROM jpcc-u20-src as jpcc-u20-build-debug

RUN mkdir -p build

RUN bash scripts/build-debug.sh

ADD ./scripts/*.sh /jason-pcc/
