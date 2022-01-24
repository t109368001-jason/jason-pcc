FROM ubuntu:20.04 as mpeg-pcc-tmc2

# disable interactive dialog
ARG TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive

# requirements
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    wget \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN wget -qO- "https://cmake.org/files/v3.17/cmake-3.17.5-Linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /usr/local

###############################################################################
FROM mpeg-pcc-tmc2 as mpeg-pcc-tmc2-src

WORKDIR /

RUN git clone --depth 1 --branch release-v15.0 https://github.com/MPEGGroup/mpeg-pcc-tmc2.git

WORKDIR /mpeg-pcc-tmc2

###############################################################################
FROM mpeg-pcc-tmc2-src as mpeg-pcc-tmc2-build

RUN mkdir build

WORKDIR /mpeg-pcc-tmc2/build

RUN cmake ..

RUN make -j$(nproc)

WORKDIR /mpeg-pcc-tmc2

ADD ./scripts/*.sh /mpeg-pcc-tmc2/
