# load("//tools/build_defs/license:license.bzl", "license")

# package(
#     default_applicable_licenses = [":license"],
# )

# license(name = "license")

# # Legacy deprecated licenses rule.
# # Should be kept until transition to new license rules is complete.
# licenses(["notice"])

# exports_files([
#     "LICENSE",
# ])

filegroup(
    name = "rtde_files",
    srcs = [
        "examples/resources/rtde_input_recipe.txt",
        "examples/resources/rtde_output_recipe.txt",
        "resources/external_control.urscript",
    ],
)

COMMON_OPTIONS = [
    "-Iinclude",
]

DEFAULT_OPTIONS = COMMON_OPTIONS + ["-fexceptions"]

PEDANTIC_OPTIONS = DEFAULT_OPTIONS + [
    "-Wall",
    "-pedantic",
]

cc_library(
    name = "ur_client_library",
    srcs = glob(
        [
            "src/**/*.cpp",
        ],
    ),
    hdrs = glob(
        [
            "include/**/*.h",
        ],
        exclude = [],
    ),
    copts = PEDANTIC_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
)

cc_binary(
    name = "full_driver",
    srcs = [
        "examples/full_driver.cpp",
    ],
    copts = PEDANTIC_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
    ],
)

cc_binary(
    name = "primary_pipeline",
    srcs = [
        "examples/primary_pipeline.cpp",
    ],
    copts = PEDANTIC_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
    ],
)

cc_binary(
    name = "primary_pipeline_calibration",
    srcs = [
        "examples/primary_pipeline_calibration.cpp",
    ],
    copts = PEDANTIC_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
    ],
)

cc_binary(
    name = "rtde_client",
    srcs = [
        "examples/rtde_client.cpp",
    ],
    copts = PEDANTIC_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
    ],
)

cc_binary(
    name = "dashboard_example",
    srcs = [
        "examples/dashboard_example.cpp",
    ],
    copts = PEDANTIC_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
    ],
)

cc_test(
    name = "test_bin_parser",
    srcs = ["tests/test_bin_parser.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_package_serializer",
    srcs = ["tests/test_package_serializer.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_primary_parser",
    srcs = ["tests/test_primary_parser.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_control_package_pause",
    srcs = ["tests/test_rtde_control_package_pause.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_control_package_setup_inputs",
    srcs = ["tests/test_rtde_control_package_setup_inputs.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_control_package_setup_outputs",
    srcs = ["tests/test_rtde_control_package_setup_outputs.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_control_package_start",
    srcs = ["tests/test_rtde_control_package_start.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_data_package",
    srcs = ["tests/test_rtde_data_package.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_get_urcontrol_version",
    srcs = ["tests/test_rtde_get_urcontrol_version.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_parser",
    srcs = ["tests/test_rtde_parser.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_request_protocol_version",
    srcs = ["tests/test_rtde_request_protocol_version.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    tags = [
        "requires-net:ipv4",
        "requires-net:loopback",
    ],
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_rtde_writer",
    srcs = ["tests/test_rtde_writer.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    tags = [
        "notsan",
        "requires-net:ipv4",
        "requires-net:loopback",
    ],
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_script_sender",
    srcs = ["tests/test_script_sender.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    tags = [
        "requires-net:ipv4",
        "requires-net:loopback",
    ],
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_stream",
    srcs = ["tests/test_stream.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    tags = [
        "notsan",
        "requires-net:ipv4",
        "requires-net:loopback",
    ],
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_tcp_server",
    srcs = ["tests/test_tcp_server.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    tags = [
        "notsan",
        "requires-net:ipv4",
        "requires-net:loopback",
    ],
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_tcp_socket",
    srcs = ["tests/test_tcp_socket.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    tags = [
        "notsan",
        "requires-net:ipv4",
        "requires-net:loopback",
    ],
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_trajectory_point_interface",
    srcs = ["tests/test_trajectory_point_interface.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    tags = [
        "notsan",
        "requires-net:ipv4",
        "requires-net:loopback",
    ],
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "test_version_information",
    srcs = ["tests/test_version_information.cpp"],
    copts =
        DEFAULT_OPTIONS,
    features = ["-use_header_modules"],  # Incompatible with -fexceptions.
    deps = [
        ":ur_client_library",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
