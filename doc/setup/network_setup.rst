:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/setup/network_setup.rst

.. _network_setup:

Network setup
=============

Robot setup
-----------

There are many possible ways to connect a UR robot. This section describes a good example using static IP addresses and a direct connection from the PC to the Robot to minimize latency introduced by network hardware. Though a good network switch usually works fine, as well.


#.
   Connect the UR control box directly to the remote PC with an ethernet cable.

#.
   Open the network settings from the UR teach pendant (Setup Robot -> Network) and enter these settings:

.. code-block::

   IP address: 192.168.56.101
   Subnet mask: 255.255.255.0
   Default gateway: 192.168.56.1
   Preferred DNS server: 192.168.56.1
   Alternative DNS server: 0.0.0.0

Remote PC setup
---------------

#. On the remote PC, turn off all network devices except the "wired connection", e.g. turn off
   wifi. You can skip this step if you are certain that no other active connection is using the
   same IP range.

#. Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection ``UR`` or something similar:

   .. code-block::

      IPv4
      Manual
      Address: 192.168.56.1
      Netmask: 255.255.255.0

#. Verify the connection from the PC with e.g. ping. You should see something like this:

   .. code-block:: console

      $ ping 192.168.56.101
      PING 192.168.56.101 (192.168.56.101) 56(84) bytes of data.
      64 bytes from 192.168.56.101: icmp_seq=1 ttl=64 time=0.037 ms
      64 bytes from 192.168.56.101: icmp_seq=2 ttl=64 time=0.043 ms
      64 bytes from 192.168.56.101: icmp_seq=3 ttl=64 time=0.047 ms
