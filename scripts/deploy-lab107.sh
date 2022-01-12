#!/bin/bash

rsync -avr --exclude 'bin/' --exclude 'build*' --exclude 'lib/' ./* lab107:~/t109368001/git/jason-pcc

ssh -t lab107 "cd t109368001/git/jason-pcc && sudo docker-compose down && sudo docker-compose up --build"