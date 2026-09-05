:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/rtde_writer.rst

.. _rtde_writer_example:

RTDE writer example
===================

This example shows how to write several `Real-Time Data Exchange (RTDE)
<https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/>`_
inputs to the robot in a single package, at the robot's maximum frequency, and how to prove that
the robot processed them.

The one-field ``send...()`` helpers on ``RTDEWriter`` each produce a package of their own. When
several general purpose registers have to change together, ``sendPackage()`` is the method that
puts them on the wire in one RTDE package.

The example's source code can be found in `rtde_writer.cpp
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/rtde_writer.cpp>`_.

.. note:: The robot has to be powered on and, on an e-Series, in *remote control mode* for the
   register-processing program to be accepted.

Recipes as argument lists
-------------------------

``RTDEClient`` takes the input and output recipes as two lists of field names. Recipe files work
as well; see :ref:`rtde_client_example`. ``timestamp`` is part of the output recipe either way,
because the client adds it if it is missing.

The general purpose register ranges reserved for external RTDE clients are bit registers
``64..127`` and integer and double registers ``24..47``.

.. literalinclude:: ../../examples/rtde_writer.cpp
   :language: c++
   :caption: examples/rtde_writer.cpp
   :linenos:
   :lineno-match:
   :start-at: const std::vector<std::string> INPUT_RECIPE
   :end-at: const std::string OUTPUT_DOUBLE_REGISTER

.. note:: Register fields, unlike the digital and analog outputs and the speed slider, need no
   companion ``_mask`` key in the input recipe.

Processing the registers on the robot
-------------------------------------

Input registers cannot be written from URScript, and output registers cannot be written through
RTDE. Getting values back therefore requires a program on the robot.

The program does not copy the values. RTDE also exposes the input registers as outputs, so a
plain echo would be indistinguishable from that read-back. Instead the program inverts the bit,
adds one to the integer and negates the double. A value that satisfies those relations can only
have been produced by this program. ``sync()`` runs the loop once per control cycle.

``sendScript()`` is used rather than ``sendScriptBlocking()``, because the latter would wait until
the program stops, and this one loops forever.

.. literalinclude:: ../../examples/rtde_writer.cpp
   :language: c++
   :caption: examples/rtde_writer.cpp
   :linenos:
   :lineno-match:
   :start-at: const std::string MIRROR_PROGRAM
   :end-at: end)";

.. literalinclude:: ../../examples/rtde_writer.cpp
   :language: c++
   :caption: examples/rtde_writer.cpp
   :linenos:
   :lineno-match:
   :start-at: // Start the robot program that processes the registers
   :end-at: // The program keeps running until we stop it later.

An input package with the robot's field types
---------------------------------------------

The data types of the input recipe belong to the robot and arrive with the handshake, so the
package has to be created after ``init()``. ``createInputDataPackage()`` returns a zeroed package
that already carries those types: ``setData()`` then rejects a wrong type immediately, and
copying the package into the send buffer is a single memcpy.

A package constructed from ``getInputRecipe()`` still works. Its types are taken from the values
written to it and are only checked when the package is sent.

.. literalinclude:: ../../examples/rtde_writer.cpp
   :language: c++
   :caption: examples/rtde_writer.cpp
   :linenos:
   :lineno-match:
   :start-at: // RTDE client at the robot's maximum frequency
   :end-at: my_client.start(false);

``target_frequency = 0.0`` (the default) requests the robot's maximum: 125 Hz on CB3, 500 Hz on
e-Series. See :ref:`real time setup` and :ref:`rtde_client`.

Both ``DataPackage`` objects are allocated before the loop, so the loop itself is allocation-free.
The output package is built from ``getOutputRecipe()`` and is therefore still untyped; the first
read applies the robot's types to it in place, which needs no memory.

Letting the robot pace the loop
-------------------------------

``start(false)`` leaves the background read thread off. ``getDataPackageBlocking()`` returns once
per RTDE cycle and is this loop's time base. The input package is produced immediately after the
read so it reaches the robot in time to be acted on in the next cycle. Printing is throttled to
about once per second, so it stays out of the hot path.

.. literalinclude:: ../../examples/rtde_writer.cpp
   :language: c++
   :caption: examples/rtde_writer.cpp
   :linenos:
   :lineno-match:
   :start-at: // The blocking read is this loop's clock
   :end-at: URCL_LOG_ERROR("Could not get a fresh data package from the robot.");

Writing several inputs in one package
-------------------------------------

Unwritten fields of the package are sent as zeros. One ``sendPackage()`` produces exactly one
RTDE package; the ``send...()`` helpers would produce one package per field. The call only queues
the values for the writer thread, so the loop stays aligned to the robot.

.. literalinclude:: ../../examples/rtde_writer.cpp
   :language: c++
   :caption: examples/rtde_writer.cpp
   :linenos:
   :lineno-match:
   :start-at: // Writing several general purpose inputs in one package
   :end-at: URCL_LOG_ERROR("Sending RTDE data failed.");

Verifying that the robot processed the data
-------------------------------------------

All three values sent in a cycle are derived from the cycle counter, so the integer the robot
returns identifies which cycle an answer belongs to. ``echoed_int - 1`` is that counter. The
expected bit is its inversion and the expected double is the negated sine. The robot's double
register is a 64-bit value, so the negated sine comes back bit for bit and is compared exactly.
Together with the inverted bit, that is what makes an answer attributable to this program rather
than to RTDE's own read-back of the input registers.

Against URSim the lag is one cycle: the values written after the read of cycle N are processed by
the robot and observed in the read of cycle N+1. ``getData()`` needs a variable of the field's own
type; ``getDataType()`` reports that type if the recipe is not known in advance.

.. literalinclude:: ../../examples/rtde_writer.cpp
   :language: c++
   :caption: examples/rtde_writer.cpp
   :linenos:
   :lineno-match:
   :start-at: // Reading what the robot made of the previous package
   :end-at: ++mismatches;

Cleanup
-------

The input registers are reset and the robot program is stopped. A failed stop is only logged,
because CI runs the example for one second and still requires exit code 0.

.. literalinclude:: ../../examples/rtde_writer.cpp
   :language: c++
   :caption: examples/rtde_writer.cpp
   :linenos:
   :lineno-match:
   :start-at: // Reset the input registers before leaving
   :end-at: return 0;

Example output
--------------

The following shows a run against URSim 5.25.1 asking for 500 Hz. The echoed integer trails the
sent integer by one cycle, and ``verified=1`` means the bit and the double match the
transformations the robot program applies to that cycle.

.. code::

   [INFO] RTDE target frequency: 500.000000 Hz
   sent: bit=1 int=484 double=-0.991869 | robot: bit=0 int=483 double=0.994216 | verified=1 lag_cycles=1 freq=483.063 Hz playing=1
   sent: bit=0 int=967 double=-0.242772 | robot: bit=1 int=966 double=0.223323 | verified=1 lag_cycles=1 freq=482.826 Hz playing=1
   sent: bit=1 int=1450 double=0.934895 | robot: bit=0 int=1449 double=-0.941806 | verified=1 lag_cycles=1 freq=482.669 Hz playing=1
   [INFO] Cycles: 1931, average frequency: 482.628400 Hz, verified: 1929, mismatches: 0, last lag: 1 cycles

A simulator shares the host's CPU, so the measured frequency stays somewhat below the requested
one; on a real controller it tracks the target closely.
