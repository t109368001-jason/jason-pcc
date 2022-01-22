#!/bin/bash
set -ex

REMOTE_HOST=lab107
REMOTE_WORKING_DIR="~/git/jason-pcc"
REMOTE_REPOSITORY_BRANCH=develop

ssh -t ${REMOTE_HOST} "
    cd ${REMOTE_WORKING_DIR} &&
    git reset --hard &&
    git checkout ${REMOTE_REPOSITORY_BRANCH} &&
    git pull &&
    bash scripts/docker-build.sh &&
    bash scripts/docker-test.sh"
