#!/bin/bash
set -ex

rsync -avrh --progress --exclude-from=.gitignore ./* lab107:~/git/jason-pcc
