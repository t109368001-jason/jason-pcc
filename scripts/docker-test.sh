#!/bin/bash
set -ex

DOCKER_FILE_ARG="--file docker/test.yml"

docker-compose ${DOCKER_FILE_ARG} down && \
    docker-compose ${DOCKER_FILE_ARG} build && \
    docker-compose ${DOCKER_FILE_ARG} up
