#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
URSIM_DOCKER_IMAGE=${1}
PROGRAM_FOLDER=${2}

docker network create --subnet=192.168.56.0/24 static_test_net

docker run --rm -it -d \
  --name ursim  \
  --net static_test_net \
  --ip 192.168.56.101 \
  -p 8080:8080 \
  -p 29999:29999 \
  -p 30001-30004:30001-30004 \
  -v "${DIR}/${PROGRAM_FOLDER}":/ursim/programs \
  --privileged \
  --cpus=1 \
  ${URSIM_DOCKER_IMAGE}
