#!/bin/bash

# copyright 2022 Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

PERSISTENT_BASE="${HOME}/.ursim"
URCAP_VERSION="1.0.5"
IP_ADDRESS="192.168.56.101"
PORT_FORWARDING="-p 30001-30004:30001-30004 -p 29999:29999"
CONTAINER_NAME="ursim"

help()
{
  # Display Help
  echo "Starts URSim inside a docker container"
  echo
  echo "Syntax: `basename "$0"` [-m|s|h]"
  echo "options:"
  echo "    -m <model>     Robot model. One of [ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30]. Defaults to ur5e."
  echo "    -v <version>   URSim version that should be used.
                   See https://hub.docker.com/r/universalrobots/ursim_e-series/tags
                   for available versions. Defaults to 'latest'"
  echo "    -p <folder>    Location from which programs are read / to which programs are written.
                   If not specified, will fallback to ${PERSISTENT_BASE}/${ROBOT_SERIES}/${ROBOT_MODEL}/programs"
  echo "    -u <folder>    Location from which URCaps are read / to which URCaps are written.
                   If not specified, will fallback to ${PERSISTENT_BASE}/${ROBOT_SERIES}/urcaps"
  echo "    -n             Name of the docker container. Defaults to '$CONTAINER_NAME'"
  echo "    -i             IP address the container should get. Defaults to $IP_ADDRESS"
  echo "    -d             Detached mode - start in background"
  echo "    -f             Specify port forwarding to use. Defaults to '$PORT_FORWARDING'. Set to empty string to disable port forwarding."
  echo "    -h             Print this Help."
  echo
}

ROBOT_MODEL=ur5e
ROBOT_SERIES=e-series
URSIM_VERSION=latest
DETACHED=false


validate_model()
{
  case $ROBOT_MODEL in
    ur3|ur5|ur10)
      ROBOT_MODEL=${ROBOT_MODEL^^}
      ROBOT_SERIES=cb3
      ;;
    ur3e|ur5e|ur10e|ur16e)
      ROBOT_MODEL=${ROBOT_MODEL^^}
      ROBOT_MODEL=$(echo ${ROBOT_MODEL:0:$((${#ROBOT_MODEL}-1))})
      ROBOT_SERIES=e-series
      ;;
    ur20|ur30)
      ROBOT_MODEL=${ROBOT_MODEL^^}
      ROBOT_SERIES=e-series
      ;;
    *)
      echo "Not a valid robot model: $ROBOT_MODEL"
      exit
      ;;
  esac
  URCAP_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/urcaps"
  PROGRAM_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/${ROBOT_MODEL}/programs"
}

verlte()
{
  [  "$1" = $(printf  "$1\n$2" | sort -V | head -n1) ]
}

validate_ursim_version()
{
  local IMAGE_URSIM_VERSION
  # Inspect the image's URSim version if the image is locally available. This is especially
  # important when we use the "latest" tag, as we don't know the version hiding behind this and it
  # could be potentially older.
  IMAGE_URSIM_VERSION=$(docker image inspect universalrobots/ursim_"${ROBOT_SERIES}":"$URSIM_VERSION" 2>/dev/null | grep -Po '"build_version": "URSim Version: \K[^"]*') || true
  if [ -z "$IMAGE_URSIM_VERSION" ]; then
    IMAGE_URSIM_VERSION="$URSIM_VERSION"
  fi
  [ "$IMAGE_URSIM_VERSION" == "latest" ] && return 0
  local MIN_CB3="3.14.3"
  local MIN_E_SERIES="5.9.4"
  local MIN_UR20="5.14.0"
  local MIN_UR30="5.15.0"

  local MIN_VERSION="0.0"


  case $ROBOT_SERIES in
    cb3)
      verlte "4.0.0" "$IMAGE_URSIM_VERSION" && echo "$IMAGE_URSIM_VERSION is no valid CB3 version!" && exit
      verlte "$MIN_CB3" "$IMAGE_URSIM_VERSION" && return 0
      MIN_VERSION=$MIN_CB3
      ;;
    e-series)
      if [[ $ROBOT_MODEL == "UR20" ]]; then
          verlte "$MIN_UR20" "$IMAGE_URSIM_VERSION" && return 0
          MIN_VERSION=$MIN_UR20
      elif [[ $ROBOT_MODEL == "UR30" ]]; then
          verlte "$MIN_UR30" "$IMAGE_URSIM_VERSION" && return 0
          MIN_VERSION=$MIN_UR30
      else
          verlte "$MIN_E_SERIES" "$URSIM_VERSION" && return 0
          MIN_VERSION=$MIN_E_SERIES
      fi
      ;;
  esac

  echo "Illegal version given. For $ROBOT_SERIES $ROBOT_MODEL the software version must be greater or equal to $MIN_VERSION. Given version: $IMAGE_URSIM_VERSION."
  exit
}


while getopts ":hm:v:p:u:i:f:n:d" option; do
  case $option in
    h) # display Help
      help
      exit;;
    m) # robot model
      ROBOT_MODEL=${OPTARG}
      ;;
    v) # ursim_version
      URSIM_VERSION=${OPTARG}
      ;;
    p) # program_folder
      PROGRAM_STORAGE_ARG=${OPTARG}
      ;;
    u) # urcaps_folder
      URCAP_STORAGE_ARG=${OPTARG}
      ;;
    i) # IP address
      IP_ADDRESS=${OPTARG}
      ;;
    f) # forward ports
      PORT_FORWARDING=${OPTARG}
      ;;
    n) # container_name
      CONTAINER_NAME=${OPTARG}
      ;;
    d) # detached mode
      DETACHED=true
      ;;
    \?) # invalid option
      echo "Error: Invalid option"
      help
      exit;;
  esac
done
validate_model
validate_ursim_version

if [ -n "$PROGRAM_STORAGE_ARG" ]; then
  PROGRAM_STORAGE="$PROGRAM_STORAGE_ARG"
fi
if [ -n "$URCAP_STORAGE_ARG" ]; then
  URCAP_STORAGE="$URCAP_STORAGE_ARG"
fi

# Create local storage for programs and URCaps
mkdir -p "${URCAP_STORAGE}"
mkdir -p "${PROGRAM_STORAGE}"
URCAP_STORAGE=$(realpath "$URCAP_STORAGE")
PROGRAM_STORAGE=$(realpath "$PROGRAM_STORAGE")

# Download external_control URCap
if [[ ! -f "${URCAP_STORAGE}/externalcontrol-${URCAP_VERSION}.jar" ]]; then
  curl -L -o "${URCAP_STORAGE}/externalcontrol-${URCAP_VERSION}.jar" \
    "https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar"
fi


# Check whether network already exists
docker network inspect ursim_net > /dev/null
if [ $? -eq 0 ]; then
  echo "ursim_net already exists"
else
  echo "Creating ursim_net"
  docker network create --subnet=192.168.56.0/24 ursim_net
fi

docker_cmd="docker run --rm -d --net ursim_net --ip $IP_ADDRESS\
  -v ${URCAP_STORAGE}:/urcaps \
  -v ${PROGRAM_STORAGE}:/ursim/programs \
  -e ROBOT_MODEL=${ROBOT_MODEL} \
  $PORT_FORWARDING \
  --name $CONTAINER_NAME \
  universalrobots/ursim_${ROBOT_SERIES}:$URSIM_VERSION || exit"

#echo $docker_cmd
$docker_cmd

# Stop container when interrupted
TRAP_CMD="
echo \"killing ursim\";
docker container kill $CONTAINER_NAME >> /dev/null;
docker container wait $CONTAINER_NAME >> /dev/null;
echo \"done\";
exit
"
trap "$TRAP_CMD" SIGINT SIGTERM

echo "Docker URSim is running"
printf "\nTo access Polyscope, open the following URL in a web browser.\n\thttp://$IP_ADDRESS:6080/vnc.html\n\n"

if [ "$DETACHED" = false ]; then
echo "To exit, press CTRL+C"
  while :
  do
    sleep 1
  done
else
  echo "To kill it, please execute 'docker stop $CONTAINER_NAME'"
fi
