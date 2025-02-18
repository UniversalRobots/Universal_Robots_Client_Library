:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/spline_example.rst

.. _spline_example:

Spline Interpolation example
============================

The URScript code inside `external_control.urscript
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/resources/external_control.urscript>`_ implements trajectory execution based upon spline interpolation of a given sequence of waypoints.

Depending on the waypoint parametrization, either cubic splines (positions, velocities, times) or
quintic splines (positions, velocities, accelerations, times) will be used.

The example in `spline_example.cpp <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/spline_example.cpp>`_ shows how to use the :ref:`trajectory_point_interface` to execute trajectories using this spline interpolation.
