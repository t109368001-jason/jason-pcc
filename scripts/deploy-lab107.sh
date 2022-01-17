#!/bin/bash

rsync -avr --exclude 'bin/' --exclude 'build*/' --exclude 'lib/' --exclude '.vscode' ./* lab107:~/t109368001/git/jason-pcc

ssh -t lab107 "cd t109368001/git/jason-pcc && docker-compose down && docker-compose up --build"
