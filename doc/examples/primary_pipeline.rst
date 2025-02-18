:github_url: https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/examples/primary_pipeline.rst

.. _primary_pipeline_example:

Primary Pipeline example
========================

This example shows how to use the ``Pipeline`` class for the robot's `primary interface
<https://www.universal-robots.com/articles/ur/interface-communication/overview-of-client-interfaces/>`_.
It reads all packages coming in from the robot't primary interface and prints their contents.

At the current time parsing primary interface data is very limited, so this example will print the
raw binary data for most package types. The example serves to demonstrate the basic control flow
used for reading data from the robot.

In this library, a "pipeline" uses a producer / consumer architecture. A producer is reading data
from a *stream*, parses that data and puts it into a *queue*. A consumer reads data from the queue
and can do whatever its purpose is.

Producer setup
--------------

To setup the producer, we need to create a stream, a parser and create a producer with those:

.. literalinclude:: ../../examples/primary_pipeline.cpp
   :language: c++
   :caption: examples/primary_pipeline.cpp
   :linenos:
   :lineno-match:
   :start-at: // First of all, we need a stream
   :end-at: prod.setupProducer();

Consumer setup
--------------

The consumer can be any consumer that is able to consume data produced by the producer, in this
case ``urcl::primary_interface::PrimaryPackage``. Here, we use a ``ShellExecutor`` that will try to
print each package's content to the shell output:

.. literalinclude:: ../../examples/primary_pipeline.cpp
   :language: c++
   :caption: examples/primary_pipeline.cpp
   :linenos:
   :lineno-match:
   :start-at: // The shell consumer
   :end-at: auto consumer = std::make_unique

Assemble the pipeline
---------------------

Finally, we need to assemble the pipeline by connecting the producer to the consumer:

.. literalinclude:: ../../examples/primary_pipeline.cpp
   :language: c++
   :caption: examples/primary_pipeline.cpp
   :linenos:
   :lineno-match:
   :start-after: auto consumer = std::make_unique
   :end-at: pipeline.run()

You can setup a custom notifier that can handle start and stop events of the pipeline. In this
example we use the basic ``INotifier`` which doesn't do anything.

With all that, we can create the ``pipeline`` by passing the producer, consumer, a name and the
notifier to it's constructor.

From this point on, the producer will read the data coming on from the stream and that data will be
processed by the consumer. We keep the example program alive for a while to see some data:

.. literalinclude:: ../../examples/primary_pipeline.cpp
   :language: c++
   :caption: examples/primary_pipeline.cpp
   :linenos:
   :lineno-match:
   :start-at: do
   :end-at: return 0


