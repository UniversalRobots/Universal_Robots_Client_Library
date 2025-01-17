|polyscope| version compatibility
=================================

The table below shows breaking changes in the library compared to |polyscope| versions. Compatibility
is listed for CB3 robots (versions 3.x.y) and e-Series robots (versions 5.x.y) respectively.

It might be possible to use the library in a more recent version than the compatible versions listed
below, but that might either require some manual modifications or not using a subset of features.
For that reason the breaking changes are also listed inside the table. Please note that choosing a
higher library version than recommended happens on your own risk and will not be supported.

If your |polyscope| version is less than the minimum required version for the latest library version,
we suggest to upgrade your robot's software. Please refer to the robot's user manual how to update
your robot.

If you don't want to update your robot you can either use the release version as listed in the
table below or checkout the latest tag before the breaking changes were introduced.

.. list-table::
   :header-rows: 1

   * - |polyscope| version
     - Max. release
     - Latest tag
     - Breaking changes
   * - < 3.12.3 / 5.5.1
     - 1.1.0
     - `polyscope_compat_break_1 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/tree/polyscope_compat_break_1>`_
     - `tcp_offset in RTDE interface <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/110>`_
   * - < 3.14.3 / 5.9.4
     - 1.3.1
     - `polyscope_compat_break_2 <https://github.com/UniversalRobots/Universal_Robots_Client_Library/tree/polyscope_compat_break_2>`_
     - `Use of scalar product in urscript <https://github.com/UniversalRobots/Universal_Robots_Client_Library/pull/151>`_

.. note::
   |polyscope| X doesn't support all features supported by this library for |polyscope| 5.
   Currently, the following components are known not to be supported:

     - Dashboard client -- |polyscope| X doesn't have a dashboard server.
     - Using external control on |polyscope| X requires another URCapX for making external control
       work. This is currently in the process of being created.
       See `Universal Robots External Control URCapX <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCapX>`_

.. |polyscope| replace:: PolyScope
