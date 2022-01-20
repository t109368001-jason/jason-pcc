#!/bin/bash
set -x

REMOTE_HOST=lab107
REMOTE_WORKING_DIR="~/git/jason-pcc"

ssh -t ${REMOTE_HOST} "
    cd ${REMOTE_WORKING_DIR} &&
    git reset --hard &&
    git pull &&
    bash scripts/docker-build.sh"
