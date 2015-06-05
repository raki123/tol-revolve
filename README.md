# Triangle of Life - Revolve (work in progress)
An implementation of the triangle of life using the Revolve robot evolution framework. It uses mostly the
default components delivered by Revolve, and implements with it the body space of [Robogen](http://www.robogen.org).
Robots are generated in the [Simulation Description Format](http://www.sd-format.org) to be simulated with
[Gazebo](http://www.gazebosim.org).

As with Revolve, the philosophy is to write only the parts that require high performance in C++, leaving the ability
to write other pieces in a language of choice (because "if you don't require performance, why would you write C++?" - 
proper attribution for this quote will follow once I find out who said it). Gazebo provides a convenient plugin
architecture and (publish / subscribe) communication framework with 
[Protobuf](https://developers.google.com/protocol-buffers/) messages that allows us to achieve just that.

Practically this means that this package provides the following:
- An implementation of Revolve's default robot architecture using Robogen's body space
- A genotype for these robots, along with a genotype => phenotype converter (relying heavily on Revolve)
- A Gazebo world and world plugin, written in C++, that gathers relevant information and publishes it
  using Gazebo's communication channels. The idea here is to have the C++ plugin do some filtering to keep
  communication and analysis in Python to a minimum.
- A Python server that basically manages the world - it keeps track of all the robots in it and communicates
  through Gazebo's channels to create new ones / destroy old ones. To do this might use the information provided
  by the world plugin and that published on other channels.