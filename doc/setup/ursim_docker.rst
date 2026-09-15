:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/setup/ursim_docker.rst

.. _ursim_docker:

Setup URSim with Docker
=======================
URSim is the offline simulator by Universal Robots. Packed into a remote or virtual machine it acts almost
identically to a real robot connected over the network. While it is possible to get URSim running
locally on a Linux system or inside a VirtualBox virtual machine, we will focus on getting things
setup using Docker. Using Docker for your simulated robot allows you to very quickly spin up a robot
testing instance with very little computational overload.

This guide will assume that you have Docker already installed and setup such that you can startup
Docker containers using your current user.

The following example will start up a simulator on a UR5e. See the Docker image `documentation <https://hub.docker.com/r/universalrobots/ursim_e-series>`_ for more details and configuration options.

Start a URSim docker container
------------------------------

To startup a simulated robot run the following command. This will start a Docker container named
``ursim`` and startup a simulated UR5e robot. It publishes ports 5900 and 6080 for VNC / browser-based
PolyScope access. Binding those ports to the loopback interface (``127.0.0.1``) keeps the GUI off your
LAN. If you omit the bind address (for example ``-p 6080:6080``), Docker listens on all interfaces and
exposes the simulator to the local network unless a firewall prevents that. You can also skip port
publishing entirely and reach PolyScope via the container IP, or bind to another interface — including
a bracketed IPv6 address such as ``-p '[::1]:6080:6080'``. See Docker's documentation on
`publish ports <https://docs.docker.com/engine/network/#published-ports>`_ for details.

.. code-block:: bash

   docker run --rm -it -p 127.0.0.1:5900:5900 -p 127.0.0.1:6080:6080 --name ursim universalrobots/ursim_e-series

With the loopback binds above, open `<http://127.0.0.1:6080/vnc.html>`_ (or connect a VNC client to
``127.0.0.1:5900``) on the Docker host.

External Control
----------------

To use the external control functionality, we will need the ``external_control`` URCap installed on
the robot and a program containing its *ExternalControl* program node. Both can be prepared on the
host machine either by creating an own Dockerfile containing those or by mounting two folders
containing installed URCaps and programs. See the Docker image `documentation <https://hub.docker.com/r/universalrobots/ursim_e-series>`_.

In this example, we will bind-mount a folder for the programs and URCaps. First, let's create a
local folder where we can store things inside:

.. code-block:: bash

   mkdir -p ${HOME}/.ursim/programs
   mkdir -p ${HOME}/.ursim/urcaps

Then, we can "install" the URCap by placing its ``.jar`` file inside the urcaps folder

.. code-block:: bash

   URCAP_VERSION=1.0.5 # latest version as if writing this
   curl -L -o ${HOME}/.ursim/urcaps/externalcontrol-${URCAP_VERSION}.jar \
     https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar

With this, start your URSim containers with the following command:

.. code-block:: bash

   docker run --rm -it -p 127.0.0.1:5900:5900 -p 127.0.0.1:6080:6080 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --name ursim universalrobots/ursim_e-series

With this, you should be able to setup the ``external_control`` URCap and create a program as
described in :ref:`URCap setup guide <install_urcap>`.

Network setup
-------------

As described above, you can always start the URSim container using the default network setup. It will most probably
always get the same IP address assigned every time, as long as you don't have any other docker containers running.
However, to make things a bit more explicit, we can setup our own docker network where we can assign a static IP
address to our URSim container.

.. code-block:: bash

   docker network create --subnet=192.168.56.0/24 ursim_net
   docker run --rm -it -p 127.0.0.1:5900:5900 -p 127.0.0.1:6080:6080 --net ursim_net --ip 192.168.56.101 universalrobots/ursim_e-series

The above commands first create a network for docker and then create a container with the URSim
image attaching to this network.

With a fixed container IP you can also skip publishing the GUI ports and open
`<http://192.168.56.101:6080/vnc.html>`_ when the browser can reach that address (typically when
Docker and the browser run on the same host). Published ports remain useful behind Docker Desktop /
NAT, or when you deliberately bind only to loopback / a specific interface.

Script startup
--------------

All of the above is put together in a script in the ``ur_client_library`` package.

By default, ``start_ursim.sh``:

* Attaches the container to ``ursim_net`` at ``192.168.56.101``
* Publishes the robot interface ports (``30001-30004``, and ``29999`` for CB3 / PolyScope 5)
* Publishes the GUI on loopback only: VNC ``127.0.0.1:5900`` / ``127.0.0.1:6080`` (CB3 / PolyScope 5),
  or the PolyScope X web UI at ``127.0.0.1:8000`` (container port ``80``)

After startup, the script prints both the container-IP URLs and the host endpoints taken from
``docker port`` for any published GUI ports. Override publishing with ``-f`` (pass ``DISABLED`` to
turn it off). Examples:

.. code-block:: bash

   # Custom host ports on all interfaces
   ./scripts/start_ursim.sh -f "-p 30001-30004:30001-30004 -p 16080:6080 -p 15900:5900"

   # Bind GUI ports to IPv6 loopback (keep the -f argument in double quotes)
   ./scripts/start_ursim.sh -f "-p 30001-30004:30001-30004 -p [::1]:6080:6080 -p [::1]:5900:5900"

.. tabs::


   .. tab:: ROS 1

      .. code-block:: bash

         rosrun ur_client_library start_ursim.sh

      This will start a URSim docker container running on ``192.168.56.101`` with the ``external_control``
      URCap preinstalled. Created programs and installation changes will be stored persistently inside
      ``${HOME}/.ursim/programs``.

      With this, you can run

      .. code-block:: bash

         roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.56.101

   .. tab:: ROS 2

      .. code-block:: bash

         ros2 run ur_client_library start_ursim.sh

      This will start a URSim docker container running on ``192.168.56.101`` with the ``external_control``
      URCap preinstalled. Created programs and installation changes will be stored persistently inside
      ``${HOME}/.ursim/programs``.

      With this, you can run

      .. code-block:: bash

         ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101

   .. tab:: Other

      If you have installed the client library from another source than ROS / ROS 2 or have
      compiled it yourself, run the ``start_ursim.sh`` script directly from the package's
      ``scripts`` folder.
