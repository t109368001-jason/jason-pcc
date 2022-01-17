#!/bin/bash
set -x

DOCKER_FILE_ARG="--file docker/test.yml"

docker-compose ${DOCKER_FILE_ARG} down && \
    docker-compose ${DOCKER_FILE_ARG} build --parallel && \
    docker-compose ${DOCKER_FILE_ARG} up
