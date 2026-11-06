:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/io_connectors.rst

.. _io_connectors:

I/O connector overview
======================

This page gives a short overview of the I/O connectors on the tool flange and the control box, and
which functions of this library act on them.

- Starting with software versions 10.15 and 5.26, URScript addresses I/O through string labels such
  as ``CB_CI0`` or ``T0_V`` (unified I/O).
- A label that does not exist on the connected hardware raises a runtime exception in URScript.
- The library reports the connected hardware as ``ControlBoxType`` (``CB5`` or ``CB7``) and
  ``ToolFlangeType`` (``V1`` or ``V2``), available from the ``PrimaryClient`` and the
  ``RTDEWriter``.
- For the complete list of labels and the connector drawings, refer to the robot manual.

Tool connectors
---------------

- **e-Series and UR-Series robots**: connector ``T0`` only. Reported as ``ToolFlangeType::V1``.
- **g-Series robots**: connectors ``T0`` to ``T3``. Reported as ``ToolFlangeType::V2``.

  - ``T0`` works the same as on e-Series and UR-Series robots.

T0 -- M8 8-pin (CB3, e-Series, UR-Series and g-Series)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Digital inputs ``T0_CI0``, ``T0_CI1``, can be configured for Safe freedrive.
- Digital outputs ``T0_DO0``, ``T0_DO1``.
- Analog inputs ``T0_AI0``, ``T0_AI1``.
- Tool power output ``T0_V``: 0, 12 or 24 V.
- Library: ``setToolVoltage()``, ``RTDEWriter::sendToolDigitalOutput()`` with pins 0-1.

T1 -- Smart Panel pogo-pin connector (g-Series only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Digital inputs ``T1_DI0`` to ``T1_DI2``.
- Digital inputs ``T1_CI0``, ``T1_CI1``, can be configured for Safe freedrive.
- Digital outputs ``T1_DO0`` to ``T1_DO3``. ``T1_DO0`` is the Smart Panel LED.
- ``T1_CONNECTED`` is true when a Smart Panel is connected and powered.
- Library: ``RTDEWriter::sendToolDigitalOutput()`` with pins 2-5.

T2 -- M8 4-pin (g-Series only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Power output only, ``T2_V``: 0, 24 or 48 V.
- Library: ``setToolVoltageT2()``, requires PolyScope X 10.15.0 or later.

T3 -- Industrial Ethernet (g-Series only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Ethernet connection to the tool.
- No I/O or power labels.

CB5 control box
---------------

- I/O:

  - Digital inputs ``CB_DI0`` to ``CB_DI7``.
  - Digital outputs ``CB_DO0`` to ``CB_DO7``.
  - Configurable digital inputs ``CB_CI0`` to ``CB_CI7`` and outputs ``CB_CO0`` to ``CB_CO7``,
    can be used for safety functions.
  - Digital inputs ``CB_DI8`` to ``CB_DI11`` with high-speed encoder input functions.
  - Analog inputs ``CB_AI0``, ``CB_AI1`` and analog outputs ``CB_AO0``, ``CB_AO1``.

- External ports:

  - One Ethernet port.
  - Two USB-A ports.

CB7 control box
---------------

- I/O:

  - Configurable digital inputs ``CB_CI0`` to ``CB_CI15`` and outputs ``CB_CO0`` to ``CB_CO15``,
    can be used for safety functions.
  - No standard digital outputs.
  - Digital inputs ``CB_DI0`` to ``CB_DI3`` with high-speed encoder input functions.
  - Analog inputs ``CB_AI0``, ``CB_AI1`` and analog outputs ``CB_AO0``, ``CB_AO1``.

- External ports:

  - Three Ethernet ports.
  - Two USB-C ports with DisplayPort output.
  - Two USB-A ports.
  - Five Ethernet ports in total, counting the end-of-arm and Teach Pendant connections.

Configuration on the Teach Pendant
----------------------------------

- Network configuration of the Ethernet ports, such as routing and firewall settings.
- Configuration of the configurable I/Os, for example whether one is used for a safety function.

Control box library functions
-----------------------------

- ``RTDEWriter::sendStandardDigitalOutput()``: sets the standard digital outputs. Returns ``false``
  on a CB7, which has none.
- ``RTDEWriter::sendConfigurableDigitalOutput()``: sets the configurable digital outputs.

  - Pins 0-7 with RTDE protocol versions below 3.
  - Pins 0-15 with RTDE protocol version 3.

- ``RTDEWriter::sendStandardAnalogOutput()``: sets the analog outputs 0-1.
