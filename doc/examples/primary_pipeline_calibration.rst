:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/primary_pipeline_calibration.rst

.. _primary_pipeline_calibration_example:

Primary Pipeline Calibration example
====================================

This example is very similar to the :ref:`primary_pipeline_example`. However, it uses a
specialized consumer that will analyze the calibration data sent from the robot instead of the
``ShellConsumer``.

Consumer setup
--------------

This example uses a specialized type of consumer that stores data about the received robot
calibration.

.. literalinclude:: ../../examples/primary_pipeline_calibration.cpp
   :language: c++
   :caption: examples/primary_pipeline_calibration.cpp
   :linenos:
   :lineno-match:
   :start-at: class CalibrationConsumer
   :end-at: };

Since the producer is reading every package from the primary interface, the consumer has to be able
to consume any primary package.

Assemble the pipeline
---------------------

The rest of the pipeline setup is the same as in the other pipeline example, just that we create a
``CalibrationConsumer`` instead of a ``ShellConsumer``:

.. literalinclude:: ../../examples/primary_pipeline_calibration.cpp
   :language: c++
   :caption: examples/primary_pipeline_calibration.cpp
   :linenos:
   :lineno-match:
   :start-at: // The calibration consumer
   :end-at: calib_pipeline.run()

Again, product handling will be happening in the background, so in the rest of the program we wait
until the calibration consumer received and processed data and then print information about that:

.. literalinclude:: ../../examples/primary_pipeline_calibration.cpp
   :language: c++
   :caption: examples/primary_pipeline_calibration.cpp
   :linenos:
   :lineno-match:
   :start-at: while (!calib_consumer
   :end-at: return 0
