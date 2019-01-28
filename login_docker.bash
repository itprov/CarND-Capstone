#!/bin/bash

docker_id=`docker ps | grep capstone | cut -d" " -f1`
docker exec -it $docker_id bash
