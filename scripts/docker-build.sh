#!/bin/sh
set -x

docker-compose down && \
    docker-compose build