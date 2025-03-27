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
PORT_FORWARDING_WITH_DASHBOARD="-p 30001-30004:30001-30004 -p 29999:29999"
PORT_FORWARDING_WITHOUT_DASHBOARD="-p 30001-30004:30001-30004"
CONTAINER_NAME="ursim"
TEST_RUN=false

# TODO: Add support for more URSim PolyScopeX versions once released
# The PolyScopeX URSim containers follow the SDK versioning scheme. This maps those to marketing
# versions
#declare -Ag POLYSCOPE_X_MAP=( ["10.7.0"]="0.12.159"
                             #["10.8.0"]="not.there.yet")
declare -Ag POLYSCOPE_X_MAP=( ["10.7.0"]="0.12.159" )

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

ROBOT_MODEL=""
ROBOT_SERIES=""
URSIM_VERSION=latest
PORT_FORWARDING=""
PROGRAM_STORAGE_ARG=""
URCAP_STORAGE_ARG=""
DETACHED=false

verlte()
{
  [  "$1" = $(printf  "$1\n$2" | sort -V | head -n1) ]
}

# Infer the robot series from a model name, e.g. ur3 -> cb3, ur5e -> e-series
# For Robots potentially bein e-series or polyscopex, this defaults to e-series
# $1 robot model e.g. ur5e
# sets ROBOT_SERIES
get_series_from_model()
{
  local robot_model=$1
  case $robot_model in
    ur3|ur5|ur10)
      ROBOT_SERIES=cb3
      ;;
    ur3e|ur5e|ur10e|ur16e)
      ROBOT_SERIES=e-series
      ;;
    ur20|ur30)
      ROBOT_SERIES=e-series
      ;;
    *)
      echo "Not a valid robot model: $robot_model"
      exit 1
      ;;
  esac
}

# Infer the robot series based on the URSim version
# uses $URSIM_VERSION
# sets ROBOT_SERIES
get_series_from_version()
{
  if [[ "$URSIM_VERSION" == "latest" ]]; then
    ROBOT_SERIES=e-series && return
  else
    verlte "10.0.0" "$URSIM_VERSION" && verlte "$URSIM_VERSION" "11.0.0" && ROBOT_SERIES=polyscopex && return
    verlte "5.0.0" "$URSIM_VERSION" && verlte "$URSIM_VERSION" "6.0.0" && ROBOT_SERIES=e-series && return
    verlte "3.0.0" "$URSIM_VERSION" && verlte "$URSIM_VERSION" "4.0.0" && ROBOT_SERIES=cb3 && return
  fi
  # If nothing above matched
  echo "URSim version $URSIM_VERSION is not supported"
  exit 1
}

# Bring the model into a format that is used internally by URSim
# $1 robot model e.g. ur5e
# $2 robot series e.g. e-series
# sets ROBOT_MODEL
strip_robot_model()
{
  local robot_model=$1
  local robot_series=$2
  if [[ "$robot_series" == "cb3" ]]; then
    ROBOT_MODEL=${robot_model^^}
  else
    ROBOT_MODEL=${robot_model^^}
    # UR20 and UR30 need no further adjustment
    if [[ "$robot_model" = @(ur3e|ur5e|ur10e|ur16e) ]]; then
      ROBOT_MODEL=$(echo "${ROBOT_MODEL:0:$((${#ROBOT_MODEL}-1))}")
    fi
  fi
}

# Make sure that all parameters match together. This checks
# - URSIM_VERSION
# - ROBOT_MODEL
# - ROBOT_SERIES
validate_parameters()
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
      MIN_VERSION=$MIN_CB3
      if [[ $ROBOT_MODEL != @(ur3|ur5|ur10) ]]; then
        echo "$ROBOT_MODEL is no valid CB3 model!" && exit 1
      fi
      verlte "4.0.0" "$IMAGE_URSIM_VERSION" && echo "$IMAGE_URSIM_VERSION is no valid CB3 version!" && exit 1
      verlte "$MIN_CB3" "$IMAGE_URSIM_VERSION" && return 0
      ;;
    e-series)
      if [[ $ROBOT_MODEL != @(ur3e|ur5e|ur10e|ur16e|ur20|ur30) ]]; then
        echo "$ROBOT_MODEL is no valid e-series model!" && exit 1
      fi
      if [[ $ROBOT_MODEL == "ur20" ]]; then
          MIN_VERSION=$MIN_UR20
      elif [[ $ROBOT_MODEL == "ur30" ]]; then
          MIN_VERSION=$MIN_UR30
      else
          MIN_VERSION=$MIN_E_SERIES
      fi
      ;;
    polyscopex)
      if [[ -z "${POLYSCOPE_X_MAP[${URSIM_VERSION}]}" ]]; then
        echo "URSim version $URSIM_VERSION is not supported"
        exit 1
      fi
      if [[ $ROBOT_MODEL != @(ur3e|ur5e|ur10e|ur16e|ur20|ur30) ]]; then
        echo "$ROBOT_MODEL is no valid PolyscopeX model!" && exit 1
      else
        return 0
      fi
      ;;
  esac

  verlte "$MIN_VERSION" "$URSIM_VERSION" && return 0

  echo "Illegal version given. For $ROBOT_SERIES $ROBOT_MODEL the software version must be greater or equal to $MIN_VERSION. Given version: $IMAGE_URSIM_VERSION."
  exit 1
}

post_setup_cb3()
{
  echo "Docker URSim is running"
  echo -e "\nTo access PolyScope, open the following URL in a web browser."
  printf "\n\n\thttp://%s:6080/vnc.html\n\n" "$IP_ADDRESS"
}
post_setup_e-series()
{
  post_setup_cb3
}

post_setup_polyscopex()
{

  echo -ne "Starting URSim. Waiting for UrService to be up..."
  curl_cmd="curl --retry-connrefused -f --write-out %{http_code} --silent --output /dev/null $IP_ADDRESS/universal-robots/urservice/api/v1/urcaps"
  status_code=$(eval "$curl_cmd")

  until [ "$status_code" == "200" ]
  do
    sleep 1
    echo -ne "."
    status_code=$(eval "$curl_cmd")
  done

  echo ""; echo "UrService is up"

  # TODO: Once we have a downloadable URCapX, we can use the following code to install it
  #urcapx_file="${HOME}/Downloads/external-control-0.1.0.urcapx"
  #echo "Installing URCapX $urcapx_file"
  #curl --location --request POST  --silent --output /dev/null "$IP_ADDRESS/universal-robots/urservice/api/v1/urcaps" --form urcapxFile=@"${urcapx_file}"
  #echo "";

  echo -e "\nTo access PolyScopeX, open the following URL in a web browser."
  printf "\n\n\thttp://%s\n\n" "$IP_ADDRESS"
}

parse_arguments(){
  while getopts ":hm:v:p:u:i:f:n:dt" option; do
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
      t) # test run
        TEST_RUN=true
        ;;
      \?) # invalid option
        echo "Error: Invalid option"
        help
        exit;;
    esac
  done
}

# As users can specify the model and / or URSim version, we can infer the rest of the parameters
# based on that. If no model is specified it should default to a ur5 / ur5e.
fill_information() {
  # If no robot model is given, set a ur5 based on the series
  if [ -z "$ROBOT_MODEL" ]; then 
    echo "No robot model given. Inferring from series"
    if [ -z "$ROBOT_SERIES" ]; then
      get_series_from_version
    fi
    if [[ "$ROBOT_SERIES" == "cb3" ]]; then
      ROBOT_MODEL=ur5
    else
      ROBOT_MODEL=ur5e
    fi
  elif [ "$URSIM_VERSION" == "latest" ]; then
    get_series_from_model "$ROBOT_MODEL"
  else
    get_series_from_version
  fi
}

test_input_handling() {
  parse_arguments "$@"
  fill_information

  echo "Running in test mode"
  echo "ROBOT_MODEL: $ROBOT_MODEL"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  echo "URSIM_VERSION: $URSIM_VERSION"

  validate_parameters
}

main() {
  parse_arguments "$@"
  

  fill_information

  echo "ROBOT_MODEL: $ROBOT_MODEL"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  echo "URSIM_VERSION: $URSIM_VERSION"

  validate_parameters

  URCAP_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/urcaps"
  PROGRAM_STORAGE="${PERSISTENT_BASE}/${ROBOT_SERIES}/${ROBOT_MODEL}/programs"

  strip_robot_model "$ROBOT_MODEL" "$ROBOT_SERIES"

  if [ -z "$PORT_FORWARDING" ]; then
    if [ "$ROBOT_SERIES" == "polyscopex" ]; then
      PORT_FORWARDING=$PORT_FORWARDING_WITHOUT_DASHBOARD
    else
      PORT_FORWARDING=$PORT_FORWARDING_WITH_DASHBOARD
    fi
  fi

  DOCKER_ARGS=""

  if [ "$ROBOT_SERIES" == "polyscopex" ]; then
    URSIM_VERSION=${POLYSCOPE_X_MAP[$URSIM_VERSION]}
    DOCKER_ARGS="$DOCKER_ARGS --privileged"
  fi

  if [ -n "$PROGRAM_STORAGE_ARG" ]; then
    PROGRAM_STORAGE="$PROGRAM_STORAGE_ARG"
  fi
  if [ -n "$URCAP_STORAGE_ARG" ]; then
    URCAP_STORAGE="$URCAP_STORAGE_ARG"
  fi

  # Check whether network already exists
  if ! docker network inspect ursim_net &> /dev/null; then
  #if [ $? -ne 0 ]; then
    echo "Creating ursim_net"
    docker network create --subnet=192.168.56.0/24 ursim_net
  fi
  if [ "$ROBOT_SERIES" == "polyscopex" ]; then
    mkdir -p "${PROGRAM_STORAGE}"
    PROGRAM_STORAGE=$(realpath "$PROGRAM_STORAGE")

    docker_cmd="docker run --rm -d \
      --net ursim_net --ip $IP_ADDRESS \
      -v ${PROGRAM_STORAGE}:/ur/bin/backend/applications \
      -e ROBOT_TYPE=${ROBOT_MODEL} \
      $PORT_FORWARDING \
      $DOCKER_ARGS \
      --name $CONTAINER_NAME \
      universalrobots/ursim_${ROBOT_SERIES}:$URSIM_VERSION"
  else
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
    docker_cmd="docker run --rm -d --net ursim_net --ip $IP_ADDRESS\
      -v ${URCAP_STORAGE}:/urcaps \
      -v ${PROGRAM_STORAGE}:/ursim/programs \
      -e ROBOT_MODEL=${ROBOT_MODEL} \
      $PORT_FORWARDING \
      --name $CONTAINER_NAME \
      universalrobots/ursim_${ROBOT_SERIES}:$URSIM_VERSION"
  fi

  if [ "$TEST_RUN" = true ]; then
    echo "$docker_cmd" | tr -s ' '
    exit
  fi
  $docker_cmd || exit 2

  # Stop container when interrupted
  TRAP_CMD="
  echo \"killing ursim\";
  docker container kill $CONTAINER_NAME >> /dev/null;
  docker container wait $CONTAINER_NAME >> /dev/null;
  echo \"done\";
  exit
  "
  trap "$TRAP_CMD" SIGINT SIGTERM


  eval "post_setup_${ROBOT_SERIES}"

  if [ "$DETACHED" = false ]; then
  echo "To exit, press CTRL+C"
    while :
    do
      sleep 1
    done
  else
    echo "To kill it, please execute 'docker stop $CONTAINER_NAME'"
  fi
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]
then
  main "$@"
fi
