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

help()
{
  # Display Help
  echo "Starts URSim inside a docker container"
  echo
  echo "Syntax: `basename "$0"` [-m|s|h]"
  echo "options:"
  echo "    -m <model>     Robot model. One of [ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20]. Defaults to ur5e."
  echo "    -v <version>   URSim version that should be used.
                   See https://hub.docker.com/r/universalrobots/ursim_e-series/tags
                   for available versions. Defaults to 'latest'"
  echo "    -p <folder>    Location from which programs are read / to which programs are written.
                   If not specified, will fallback to ${PERSISTENT_BASE}/${ROBOT_SERIES}/programs"
  echo "    -u <folder>    Location from which URCaps are read / to which URCaps are written.
                   If not specified, will fallback to ${PERSISTENT_BASE}/${ROBOT_SERIES}/urcaps"
  echo "    -d     Detached mode - start in backgound"
  echo "    -h             Print this Help."
  echo
}

ROBOT_MODEL=UR5
ROBOT_SERIES=e-series
URSIM_VERSION=latest
URCAP_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/urcaps"
PROGRAM_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/programs"
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
    ur20)
      ROBOT_MODEL=${ROBOT_MODEL^^}
      ROBOT_SERIES=e-series
      ;;
    *)
      echo "Not a valid robot model: $ROBOT_MODEL"
      exit
      ;;
  esac
}

verlte()
{
  [  "$1" = $(printf  "$1\n$2" | sort -V | head -n1) ]
}

validate_ursim_version()
{
  [ $URSIM_VERSION == "latest" ] && return 0
  local MIN_CB3="3.14.3"
  local MIN_E_SERIES="5.9.4"
  local MIN_UR20="5.14.0"

  local MIN_VERSION="0.0"


  case $ROBOT_SERIES in
    cb3)
      verlte "4.0.0" $URSIM_VERSION && echo "$URSIM_VERSION is no valid CB3 version!" && exit
      verlte $MIN_CB3 $URSIM_VERSION && return 0
      MIN_VERSION=$MIN_CB3
      ;;
    e-series)
      if [[ $ROBOT_MODEL == "UR20" ]]; then
          verlte $MIN_UR20 $URSIM_VERSION && return 0
          MIN_VERSION=$MIN_UR20
      else
          verlte $MIN_E_SERIES $URSIM_VERSION && return 0
          MIN_VERSION=$MIN_E_SERIES
      fi
      ;;
  esac

  echo "Illegal version given. Version must be greater or equal to $MIN_VERSION. Given version: $URSIM_VERSION."
  exit
}


while getopts ":hm:v:p:u:d" option; do
  case $option in
    h) # display Help
      help
      exit;;
    m) # robot model
      ROBOT_MODEL=${OPTARG}
      validate_model
      ;;
    v) # ursim_version
      URSIM_VERSION=${OPTARG}
      validate_ursim_version
      ;;
    p) # program_folder
      PROGRAM_STORAGE=${OPTARG}
      ;;
    u) # urcaps_folder
      URCAP_STORAGE=${OPTARG}
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

# run docker contain
docker run --rm -d --net ursim_net --ip 192.168.56.101\
  -v "${URCAP_STORAGE}":/urcaps \
  -v "${PROGRAM_STORAGE}":/ursim/programs \
  -e ROBOT_MODEL="${ROBOT_MODEL}" \
  -p 30001-30004:30001-30004 \
  -p 29999:29999 \
  --name ursim \
  universalrobots/ursim_${ROBOT_SERIES}:$URSIM_VERSION || exit

trap "echo killing; docker container kill ursim; exit" SIGINT SIGTERM

echo "Docker URSim is running"
printf "\nTo access Polyscope, open the following URL in a web browser.\n\thttp://192.168.56.101:6080/vnc.html\n\n"

if [ "$DETACHED" = false ]; then
echo "To exit, press CTRL+C"
  while :
  do
    sleep 1
  done
else
  echo "To kill it, please execute 'docker stop ursim'"
fi
