FROM ubuntu:20.04

# disable interactive dialog
ARG TZ=Etc/UTC
ARG DEBIAN_FRONTEND=noninteractive

# requirements
RUN apt-get update
RUN apt-get install -y build-essential
RUN apt-get install -y cmake
RUN apt-get install -y valgrind
RUN apt-get install -y libpcap-dev
RUN apt-get install -y libtbb-dev
RUN apt-get install -y libpcl-dev

COPY ./ /jason-pcc

WORKDIR /jason-pcc

RUN mkdir -p build
