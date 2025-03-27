setup() {
    # get the containing directory of this file
    # use $BATS_TEST_FILENAME instead of ${BASH_SOURCE[0]} or $0,
    # as those will point to the bats executable's location or the preprocessed file respectively
    DIR="$( cd "$( dirname "$BATS_TEST_FILENAME" )" >/dev/null 2>&1 && pwd )"
    # make executables in src/ visible to PATH
    source $DIR/../scripts/start_ursim.sh
}

@test "test get_series_from_model" {
  get_series_from_model "ur10e"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  [ "$ROBOT_SERIES" = "e-series" ]

  get_series_from_model "ur3"
  echo "ROBOT_SERIES: $ROBOT_SERIES"
  [ "$ROBOT_SERIES" = "cb3" ]

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

@test "docker image polyscopex" {
  run main -v 10.7.0 -t
  echo "$output"
  image=$(echo "$output" | tail -n1 | awk '{ print $NF }')
  [ $status -eq 0 ]
  [ "$image" == "universalrobots/ursim_polyscopex:0.12.159" ]
}

@test "docker image cb3 latest" {
  run main -m ur3 -t
  echo "$output"
  image=$(echo "$output" | tail -n1 | awk '{ print $NF }')
  [ $status -eq 0 ]
  [ "$image" == "universalrobots/ursim_cb3:latest" ]
}

@test "docker image e-series latest" {
  run main -m ur3e -t
  echo "$output"
  image=$(echo "$output" | tail -n1 | awk '{ print $NF }')
  [ $status -eq 0 ]
  [ "$image" == "universalrobots/ursim_e-series:latest" ]
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
}
