#!/bin/bash
set -x

REMOTE_HOST=lab107
REMOTE_WORKING_DIR="~/t109368001/git/jason-pcc"

ssh -t ${REMOTE_HOST} "rm ${REMOTE_WORKING_DIR} -rf"

rsync -avr --exclude-from ".gitignore" ./* ${REMOTE_HOST}:${REMOTE_WORKING_DIR}

ssh -t ${REMOTE_HOST} "
    cd ${REMOTE_WORKING_DIR} && 
    bash scripts/docker-test.sh"
