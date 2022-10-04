# Polyscope version compatibility
The table below shows breaking changes in the library compared to Polyscope versions. Compatibility
is listed for CB3 robots (versions 3.x.y) and e-Series robots (versions 5.x.y) respectively.

It might be possible to use the library in a more recent version than the compatible versions listed
below, but that might either require some manual modifications or not using a subset of features.
For that reason the breaking changes are also listed inside the table. Please note that choosing a
higher library version than recommended happens on your own risk and will not be supported.

If your Polyscope version is less than the minimum required version for the latest library version,
we suggest to upgrade your robot's software. Please refer to the robot's user manual how to update
your robot.


|Polyscope version | Maximum tag | Breaking changes |
|------------------|-------------|------------------|
| < 3.12.0 / 5.5.1 | [polyscope_compat_break_1](https://github.com/UniversalRobots/Universal_Robots_Client_Library/tree/polyscope_compat_break_1) | [tcp_offset in RTDE interface](https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/110)|
