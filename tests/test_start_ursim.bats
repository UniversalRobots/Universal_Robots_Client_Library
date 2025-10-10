setup_file() {
  echo "Pulling latest URSim Docker images for CB3 and PolyScope 5"
  docker pull universalrobots/ursim_cb3:latest
  docker pull universalrobots/ursim_e-series:latest
}

setup() {
    # get the containing directory of this file
    # use $BATS_TEST_FILENAME instead of ${BASH_SOURCE[0]} or $0,
    # as those will point to the bats executable's location or the preprocessed file respectively
    DIR="$( cd "$( dirname "$BATS_TEST_FILENAME" )" >/dev/null 2>&1 && pwd )"
    # make executables in src/ visible to PATH
    source $DIR/../scripts/start_ursim.sh
}

@test "test_get_version_from_release_url" {
  effective_url="https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCapX/releases/0.1.0"

  VERSION=$(get_version_from_release_url "$effective_url")

  [ "$VERSION" = "0.1.0" ]
}

@test "test_get_download_url_urcapx" {
  get_download_url_urcapx "0.1.0"
  echo "Download URL: $URCAPX_DOWNLOAD_URL"
  echo "URCAPX_VERSION: $URCAPX_VERSION"
  [ "$URCAPX_DOWNLOAD_URL" = "https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCapX/releases/download/0.1.0/external-control-0.1.0.urcapx" ]
}

@test "test_get_download_url_urcap" {
  get_download_url_urcap "1.0.5"
  echo "Download URL: $URCAP_DOWNLOAD_URL"
  echo "URCAP_VERSION: $URCAP_VERSION"
  [ "$URCAP_DOWNLOAD_URL" = "https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v1.0.5/externalcontrol-1.0.5.jar" ]
}

@test "test get_series_from_model" {
  get_series_from_model "ur10e"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  [ "$ROBOT_SERIES" = "e-series" ]

  get_series_from_model "ur3"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  [ "$ROBOT_SERIES" = "cb3" ]

  get_series_from_model "ur15"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  [ "$ROBOT_SERIES" = "e-series" ]

  get_series_from_model "ur20"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  [ "$ROBOT_SERIES" = "e-series" ]

  get_series_from_model "ur30"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  [ "$ROBOT_SERIES" = "e-series" ]

  run get_series_from_model "notarobotname"
  [ $status -eq 1 ]
}

@test "latest cb3" {
  test_input_handling -m ur3
  [ "$ROBOT_MODEL" = "ur3" ]
  [ "$ROBOT_SERIES" = "cb3" ]
  [ "$URSIM_VERSION" = "latest" ]
}

@test "specific cb3" {
  test_input_handling -m ur10 -v 3.14.3
  [ "$ROBOT_MODEL" = "ur10" ]
  [ "$ROBOT_SERIES" = "cb3" ]
  [ "$URSIM_VERSION" = "3.14.3" ]
}

@test "cb3_version too low fails" {
  run test_input_handling -m ur10 -v 3.10.1
  echo "$output"
  [ $status -eq 1 ]
}

@test "cb3 version with e-series-model fails" {
  run test_input_handling -m ur10e -v 3.12.1
  echo "$output"
  [ $status -eq 1 ]

  run test_input_handling -m ur20 -v 3.12.1
  echo "$output"
  [ $status -eq 1 ]
}

@test "cb3 series from version string" {
  test_input_handling -v 3.14.3
  [ "$ROBOT_SERIES" = "cb3" ]
}

@test "latest e-series" {
  test_input_handling -m ur5e
  [ "$ROBOT_MODEL" = "ur5e" ]
  [ "$ROBOT_SERIES" = "e-series" ]
  [ "$URSIM_VERSION" = "latest" ]
}

@test "specific e-series" {
  test_input_handling -m ur10e -v 5.18.0
  [ "$ROBOT_MODEL" = "ur10e" ]
  [ "$ROBOT_SERIES" = "e-series" ]
  [ "$URSIM_VERSION" = "5.18.0" ]
}

@test "e-series too low fails" {
  run test_input_handling -m ur10e -v 5.1.0
  echo "$output"
  [ $status -eq 1 ]
}

@test "e-series version with cb3 model fails" {
  run test_input_handling -m ur10 -v 5.21.0
  echo "$output"
  [ $status -eq 1 ]
}

@test "e-series series from version string" {
  test_input_handling -v 5.18.0
  [ "$ROBOT_SERIES" = "e-series" ]
}


@test "specific polyscopex" {
  test_input_handling -m ur10e -v 10.7.0
  [ "$ROBOT_MODEL" = "ur10e" ]
  [ "$ROBOT_SERIES" = "polyscopex" ]
  [ "$URSIM_VERSION" = "10.7.0" ]
}

@test "polyscopex too low fails" {
  run test_input_handling -m ur10e -v 10.1.0
  echo "$output"
  [ $status -eq 1 ]
}

@test "polyscopex version with cb3 model fails" {
  run test_input_handling -m ur10 -v 10.7.0
  echo "$output"
  [ $status -eq 1 ]
}

@test "polyscopex series from version string" {
  test_input_handling -v 10.7.0
  [ "$ROBOT_SERIES" = "polyscopex" ]
}

@test "no arguments results in e-series ur5e" {
  test_input_handling
  [ "$ROBOT_MODEL" = "ur5e" ]
  [ "$ROBOT_SERIES" = "e-series" ]
  [ "$URSIM_VERSION" = "latest" ]
}

@test "test ur7e min version" {
  run test_input_handling -m ur7e -v 3.14.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur7e -v 5.21.0
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur7e -v 10.8.0
  echo "$output"
  [ $status -eq 1 ]

  run test_input_handling -m ur7e -v 5.22.0
  echo "$output"
  [ $status -eq 0 ]

  run test_input_handling -m ur7e -v 10.9.0
  echo "$output"
  [ $status -eq 0 ]
}

@test "test ur12e min version" {
  run test_input_handling -m ur12e -v 3.14.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur12e -v 5.21.0
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur12e -v 10.8.0
  echo "$output"
  [ $status -eq 1 ]

  run test_input_handling -m ur12e -v 5.22.0
  echo "$output"
  [ $status -eq 0 ]

  run test_input_handling -m ur12e -v 10.9.0
  echo "$output"
  [ $status -eq 0 ]
}

@test "test ur15 min version" {
  run test_input_handling -m ur15 -v 3.14.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur15 -v 5.21.0
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur15 -v 10.7.0
  echo "$output"
  [ $status -eq 1 ]

  run test_input_handling -m ur15 -v 5.22.0
  echo "$output"
  [ $status -eq 0 ]
  run test_input_handling -m ur15 -v 10.8.0
  echo "$output"
  [ $status -eq 0 ]
}

@test "test ur20 min version" {
  run test_input_handling -m ur20 -v 3.14.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur20 -v 5.13.9
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur20 -v 5.14.0
  echo "$output"
  [ $status -eq 0 ]
  run test_input_handling -m ur20 -v 10.7.0
  echo "$output"
  [ $status -eq 0 ]
}

@test "test ur30 min version" {
  run test_input_handling -m ur30 -v 3.14.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur30 -v 5.14.9
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur30 -v 5.15.0
  echo "$output"
  [ $status -eq 0 ]
  run test_input_handling -m ur30 -v 10.7.0
  echo "$output"
  [ $status -eq 0 ]
}

@test "test ur8long min version" {
  run test_input_handling -m ur8long -v 3.14.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur8long -v 5.22.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur8long -v 5.23.0
  echo "$output"
  [ $status -eq 0 ]
  run test_input_handling -m ur8long -v 10.10.0
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur8long -v 10.11.0
  echo "$output"
  [ $status -eq 0 ]
}

@test "test ur18 min version" {
  run test_input_handling -m ur18 -v 3.14.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur18 -v 5.22.3
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur18 -v 5.23.0
  echo "$output"
  [ $status -eq 0 ]
  run test_input_handling -m ur18 -v 10.10.0
  echo "$output"
  [ $status -eq 1 ]
  run test_input_handling -m ur18 -v 10.11.0
  echo "$output"
  [ $status -eq 0 ]
}

@test "unsupported versions raise error" {
  run main -v 1.2.3 -t
  echo "$output"
  [ $status -eq 1 ]
  run main -v 2.0.0 -t
  echo "$output"
  [ $status -eq 1 ]
  run main -v 10.1.0 -t
  echo "$output"
  [ $status -eq 1 ]
  run main -v 6.99.123 -t
  echo "$output"
  [ $status -eq 1 ]
}

@test "docker image cb3 latest" {
  run main -m ur3 -t
  echo "$output"
  image=$(echo "$output" | tail -n1 | awk '{ print $NF }')
  [ $status -eq 0 ]
  [[ "$image" =~ universalrobots/ursim_cb3:[0-9]+\.[0-9]+\.[0-9]+ ]]
}

@test "docker image e-series latest" {
  run main -m ur3e -t
  echo "$output"
  image=$(echo "$output" | tail -n1 | awk '{ print $NF }')
  [ $status -eq 0 ]
  [[ "$image" =~ universalrobots/ursim_e-series:[0-9]+\.[0-9]+\.[0-9]+ ]]
}

@test "docker image cb3 specific" {
  run main -v 3.14.3 -t
  echo "$output"
  image=$(echo "$output" | tail -n1 | awk '{ print $NF }')
  [ $status -eq 0 ]
  [ "$image" == "universalrobots/ursim_cb3:3.14.3" ]
}

@test "docker image e-series specific" {
  run main -v 5.21.0 -t
  echo "$output"
  image=$(echo "$output" | tail -n1 | awk '{ print $NF }')
  [ $status -eq 0 ]
  [ "$image" == "universalrobots/ursim_e-series:5.21.0" ]
}

@test "strip robot model cb3" {
  run main -m ur3 -t
  echo "$output"
  [ $status -eq 0 ]
  model=$(echo "$output" | tail -n1 | grep -Po '-e ROBOT_MODEL=\w+' | cut -d '=' -f2)
  [ "$model" == "UR3" ]

  run main -m ur10 -t
  echo "$output"
  [ $status -eq 0 ]
  model=$(echo "$output" | tail -n1 | grep -Po '-e ROBOT_MODEL=\w+' | cut -d '=' -f2)
  [ "$model" == "UR10" ]
}

@test "strip robot model e-series" {
  run main -m ur3e -v 5.21.0 -t
  echo "$output"
  [ $status -eq 0 ]
  model=$(echo "$output" | tail -n1 | grep -Po '-e ROBOT_MODEL=\w+' | cut -d '=' -f2)
  [ "$model" == "UR3" ]

  run main -m ur10e -v 5.21.0 -t
  echo "$output"
  [ $status -eq 0 ]
  model=$(echo "$output" | tail -n1 | grep -Po '-e ROBOT_MODEL=\w+' | cut -d '=' -f2)
  [ "$model" == "UR10" ]

  run main -m ur30 -v 5.21.0 -t
  echo "$output"
  [ $status -eq 0 ]
  model=$(echo "$output" | tail -n1 | grep -Po '-e ROBOT_MODEL=\w+' | cut -d '=' -f2)
  [ "$model" == "UR30" ]
}

@test "strip robot model polyscopex" {
  run main -m ur3e -v 10.7.0 -t
  echo "$output"
  [ $status -eq 0 ]
  model=$(echo "$output" | tail -n1 | grep -Po '-e ROBOT_TYPE=\w+' | cut -d '=' -f2)
  [ "$model" == "UR3" ]

  run main -m ur10e -v 10.7.0 -t
  echo "$output"
  [ $status -eq 0 ]
  model=$(echo "$output" | tail -n1 | grep -Po '-e ROBOT_TYPE=\w+' | cut -d '=' -f2)
  [ "$model" == "UR10" ]

  run main -m ur30 -v 10.7.0 -t
  echo "$output"
  [ $status -eq 0 ]
  model=$(echo "$output" | tail -n1 | grep -Po '-e ROBOT_TYPE=\w+' | cut -d '=' -f2)
  [ "$model" == "UR30" ]
}

@test "strip_robot_model" {
  strip_robot_model ur3 cb3
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR3" ]

  strip_robot_model ur3e e-series
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR3" ]

  strip_robot_model ur3e polyscopex
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR3" ]

  strip_robot_model ur20 e-series
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR20" ]

  strip_robot_model ur30 e-series
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR30" ]

  strip_robot_model ur20 polyscopex
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR20" ]

  strip_robot_model ur30 polyscopex
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR30" ]

  strip_robot_model ur7e e-series
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR7e" ]

  strip_robot_model ur12e e-series
  echo "Robot model is: $ROBOT_MODEL"
  [ "$ROBOT_MODEL" = "UR12e" ]
}

@test "help_prints_fine" {
  run main -h
  [ $status -eq 0 ]
}

@test "setting_urcap_storage" {
  run main -t
  echo "$output"
  [ $status -eq 0 ]
  urcap_mount=$(echo "$output" | tail -n1 | grep -Po "\-v\ [\/\w+\.\-]+:\/urcaps" | cut -d ':' -f1 | cut -d " " -f 2)
  [ "$urcap_mount" = "$HOME/.ursim/e-series/urcaps" ]

  run main -t -v 3.14.3
  echo "$output"
  [ $status -eq 0 ]
  urcap_mount=$(echo "$output" | tail -n1 | grep -Po "\-v\ [\/\w+\.\-]+:\/urcaps" | cut -d ':' -f1 | cut -d " " -f 2)
  [ "$urcap_mount" = "$HOME/.ursim/cb3/urcaps" ]

  target_dir=$(mktemp -d)
  run main -u "$target_dir" -t
  echo "$output"
  [ $status -eq 0 ]
  urcap_mount=$(echo "$output" | tail -n1 | grep -Po "\-v\ [\/\w+\.\-]+:\/urcaps" | cut -d ':' -f1 | cut -d " " -f 2)
  [ "$urcap_mount" = "$target_dir" ]

}

@test "setting_program_storage" {
  run main -t
  echo "$output"
  [ $status -eq 0 ]
  program_mount=$(echo "$output" | tail -n1 | grep -Po "\-v\ [\/\w+\.\-]+:\/ursim\/programs" | cut -d ':' -f1 | cut -d " " -f 2)
  [ "$program_mount" = "$HOME/.ursim/e-series/ur5e/programs" ]

  run main -t -v 3.14.3
  echo "$output"
  [ $status -eq 0 ]
  program_mount=$(echo "$output" | tail -n1 | grep -Po "\-v\ [\/\w+\.\-]+:\/ursim\/programs" | cut -d ':' -f1 | cut -d " " -f 2)
  [ "$program_mount" = "$HOME/.ursim/cb3/ur5/programs" ]

  run main -t -v 10.7.0
  echo "$output"
  [ $status -eq 0 ]
  program_mount=$(echo "$output" | tail -n1 | grep -Po "\-v\ [\/\w+\.\-]+:\/ur\/bin\/backend\/applications" | cut -d ':' -f1 | cut -d " " -f 2)
  [ "$program_mount" = "$HOME/.ursim/polyscopex/ur5e/programs" ]

  target_dir=$(mktemp -d)
  run main -p "$target_dir" -t
  echo "$output"
  [ $status -eq 0 ]
  program_mount=$(echo "$output" | tail -n1 | grep -Po "\-v\ [\/\w+\.\-]+:\/ursim\/programs" | cut -d ':' -f1 | cut -d " " -f 2)
  [ "$program_mount" = "$target_dir" ]
}

@test "setting_ip_addresss" {
  run main -t -i 123.123.123.123
  echo "$output"
  [ $status -eq 0 ]
  ip_address=$(echo "$output" | tail -n1 | grep -o -E "\-\-ip ([0-9]+\.?)+" | cut -d " " -f2)
  [ "$ip_address" = "123.123.123.123" ]
}

@test "default_port_forwarding_cb3" {
  run main -t -v 3.14.3
  echo "$output"
  [ $status -eq 0 ]
  port_forwarding=$(echo "$output" | tail -n -1 | grep -Eo "(\-p\s*[0-9]+(\-[0-9]+)?:[0-9]+(\-[0-9]+)?\s*)+" | awk '{$1=$1};1')
  [ "$port_forwarding" = "$PORT_FORWARDING_WITH_DASHBOARD" ]
}

@test "default_port_forwarding_e-series" {
  run main -t -v 5.21.0
  echo "$output"
  [ $status -eq 0 ]
  port_forwarding=$(echo "$output" | tail -n -1 | grep -Eo "(\-p\s*[0-9]+(\-[0-9]+)?:[0-9]+(\-[0-9]+)?\s+)+" | awk '{$1=$1};1')
  [ "$port_forwarding" = "$PORT_FORWARDING_WITH_DASHBOARD" ]
}

@test "default_port_forwarding_polyscopex" {
  run main -t -v 10.7.0
  echo "$output"
  [ $status -eq 0 ]
  port_forwarding=$(echo "$output" | tail -n -1 | grep -Eo "(\-p\s*[0-9]+(\-[0-9]+)?:[0-9]+(\-[0-9]+)?\s*)+" | awk '{$1=$1};1')
  [ "$port_forwarding" = "$PORT_FORWARDING_WITHOUT_DASHBOARD" ]
}

@test "setting_port_forwarding" {
  run main -t -f "-p 1234:1234 -p 50001-50004:60001-60004"
  echo "$output"
  [ $status -eq 0 ]
  port_forwarding=$(echo "$output" | tail -n -1 | grep -Eo "(\-p\s*[0-9]+(\-[0-9]+)?:[0-9]+(\-[0-9]+)?\s*)+" | awk '{$1=$1};1')
  [ "$port_forwarding" = "-p 1234:1234 -p 50001-50004:60001-60004" ]
}

@test "default_container_name" {
  run main -t
  echo "$output"
  [ $status -eq 0 ]
  container_name=$(echo "$output" | tail -n -1 | grep -o -E "\-\-name\s\w+" | cut -d " " -f2)
  [ "$container_name" = "ursim" ]
}

@test "setting_container_name_cb3" {
  run main -t -n "ursim_test" -v 3.14.3
  echo "$output"
  [ $status -eq 0 ]
  container_name=$(echo "$output" | tail -n -1 | grep -o -E "\-\-name\s\w+" | cut -d " " -f2)
  [ "$container_name" = "ursim_test" ]
}

@test "setting_container_name_e-series" {
  run main -t -n "ursim_test" -v 5.21.0
  echo "$output"
  [ $status -eq 0 ]
  container_name=$(echo "$output" | tail -n -1 | grep -o -E "\-\-name\s\w+" | cut -d " " -f2)
  [ "$container_name" = "ursim_test" ]
}

@test "setting_container_name_polyscopex" {
  run main -t -n "ursim_test" -v 10.7.0
  echo "$output"
  [ $status -eq 0 ]
  container_name=$(echo "$output" | tail -n -1 | grep -o -E "\-\-name\s\w+" | cut -d " " -f2)
  [ "$container_name" = "ursim_test" ]
}

@test "catch_unknown_parameters" {
  run main -t -x
  echo "$output"
  [ $status -eq 1 ]
}

@test "not_detached_by_default" {
  test_input_handling
  [ "$DETACHED" = "false" ]
}

@test "setting_detached_argument" {
  test_input_handling -d
  [ "$DETACHED" = "true" ]
}

@test "successful_validate_parameters" {
  URSIM_VERSION="5.21.0"
  ROBOT_MODEL="ur10e"
  ROBOT_SERIES="e-series"

  validate_parameters
}

@test "successful_validate_parameters_latest_e" {
  URSIM_VERSION="latest"
  ROBOT_MODEL="ur10e"
  ROBOT_SERIES="e-series"

  validate_parameters
}

@test "validate_parameters_on_invalid_model_fails_e" {
  URSIM_VERSION="latest"
  ROBOT_MODEL="ur10"
  ROBOT_SERIES="e-series"

  run validate_parameters
  [ $status -eq 1 ]
}

@test "successful_validate_parameters_latest_cb3" {
  URSIM_VERSION="latest"
  ROBOT_MODEL="ur10"
  ROBOT_SERIES="cb3"

  validate_parameters
}

@test "validate_parameters_on_invalid_model_fails_cb3" {
  URSIM_VERSION="latest"
  ROBOT_MODEL="ur103"
  ROBOT_SERIES="cb3"

  run validate_parameters
  [ $status -eq 1 ]
}

@test "validate_parameters_on_invalid_version_fails" {
  URSIM_VERSION="foobar"
  ROBOT_MODEL="ur10e"
  ROBOT_SERIES="e-series"

  run validate_parameters
  [ $status -eq 1 ]
}

@test "validate_parameters_on_invalid_model_fails" {

  URSIM_VERSION="5.21.0"
  ROBOT_MODEL="ur10"
  ROBOT_SERIES="e-series"
  run validate_parameters
  [ $status -eq 1 ]

}

