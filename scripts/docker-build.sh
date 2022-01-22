#!/bin/sh
set -ex

docker-compose down && \
    docker-compose build