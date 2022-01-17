#!/bin/sh
set -x

docker-compose down && \
    docker-compose build --parallel && \
    docker-compose run -d jpcc-u20