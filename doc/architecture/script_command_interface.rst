:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/script_command_interface.rst

.. _script_command_interface:

Script Command Interface
========================

The ``ScriptCommandInterface`` allows using functionality of the robot that would normally
interrupt program execution. For this purpose a subset of the robot's functionality is made
available in specific API calls.

At the time of writing the ``ScriptCommandInterface`` provides the following functions:

- ``zeroFTSensor()``: Zeros the force/torque sensor.
- ``setPayload()``: Set the active payload mass and center of gravity.
- ``setToolVoltage()``: Set the voltage of the tool output.
- ``startForceMode()`` / ``endForceMode()``: Start and end a force mode. See the `force mode
  example <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/force_mode_example.cpp>`_ for more information.
- ``startToolContact()`` / ``endToolContact()``: Start and end a tool contact mode. See the `tool
  contact example
  <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/examples/tool_contact_example.cpp>`_
  for more information.
- ``setFrictionCompensation()``: Set friction compensation for torque command.
- ``ftRtdeInputEnable()``: Enable/disable FT RTDE input processing.

Communication protocol
----------------------

The ``ScriptCommandInterface`` communication is part of the protocol implemented in the
`external_control.urscript
<https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/resources/external_control.urscript>`_.

Data sent to the robot
^^^^^^^^^^^^^^^^^^^^^^

The robot reads from the "script_command_socket" expecting a 32 bit integer representation of up to
28 datafields.

.. table:: script_command_socket to_robot message format
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   0      script command functionality. Can be either of
           - 0: zeroFTSensor
           - 1: setPayload
           - 2: setToolVoltage
           - 3: startForceMode
           - 4: endForceMode
           - 5: startToolContact
           - 6: endToolContact
           - 7: setFrictionCompensation
           - 8: ftRtdeInputEnable
   1-27   data fields specific to the command
   =====  =====

.. table:: With zeroFTSensor command
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1-27   No specific meaning / values ignored
   =====  =====

.. table:: With setPayload command
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1      Payload mass in kg (floating point)
   2-4    Payload center of gravity in m, displacement from the toolmpount (floating point)
   =====  =====

.. table:: With setToolVoltage command
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1      Voltage in V (Has to be 0, 12 or 24)
   =====  =====

.. table:: With startForceMode command
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1-6    task frame (floating point, see script manual for details)
   7-12   selection vector (floating point, see script manual for details)
   13-18  wrench (floating point, see script manual for details)
   19     force_type(one of 1, 2 and 3, see script manual for details)
   20-25  limits (floating point, see script manual for details)
   26     damping_factor (floating point, see script manual for details)
   27     gain_scaling (not on CB3 robots) (floating point, see script manual for details)
   =====  =====

.. table:: With endForceMode command
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1      No specific meaning / values ignored
   =====  =====

.. table:: With startToolContact command
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1      No specific meaning / values ignored
   =====  =====

.. table:: With endToolContact command
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1      No specific meaning / values ignored
   =====  =====

.. table:: With setFrictionCompensation command
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1      friction_compensation_enabled enable/disable friction compensation for torque command.
   =====  =====

.. table:: With ftRtdeInputEnable command. See script manual for details.
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   1      ft_rtde_input_enabled enable/disable FT RTDE input processing.
   2      sensor_mass in kg (floating point)
   3-5    sensor_mesurement_offset in m, displacement from the tool flange (3d floating point)
   6-9    sensor_cog in m, displacement from the tool flange (3d floating point)
   =====  =====

.. note::
   In URScript the ``socket_read_binary_integer()`` function is used to read the data from the
   script command socket. The first index in that function's return value is the number of integers read,
   so the actual data starts at index 1. The first data entry is always the command type, hence the
   indices in the table above are shifted by one when accessing the result array of the URScript
   function. E.g. reading the boolean value for friction compensation in the
   ``setFrictionCompensation`` command would be done by accessing index 2 of the result array.

   All floating point data is encoded into an integer representation and has to be divided by the
   ``MULT_JOINTSTATE`` constant to get the actual floating point value. This constant is defined in
   ``ReverseInterface`` class.

Data sent from the robot
^^^^^^^^^^^^^^^^^^^^^^^^

Data is being sent from the robot to the ``ScriptCommandInterface`` only when tool contact is used.
It will send either ``UNTIL_TOOL_CONTACT_RESULT_SUCCESS`` when tool contact has been established while tool contact was active or ``UNTIL_TOOL_CONTACT_RESULT_CANCELED`` if tool contact mode was ended without establishing physical contact.
