#!/bin/bash

docker-compose down && docker-compose build --parallel && docker-compose up -d
docker-compose logs -f jpcc-u20-test-release jpcc-u20-test-debug
