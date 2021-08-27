[for Japanese:日本語](README_JP.md)

# Overview
This is information, documentation or tips of ROS2.<br>
At first, see [tutorial in official site(link is for foxy)](https://docs.ros.org/en/foxy/Tutorials.html).
I recomend it not only because it is official information, but also because it is easy to imagine a mechanism of ROS2, to understand basic coding technique.

## Basic setting and terms

I use a following directory as top one of ROS2.

* ~/program/ros2

Under this directory, workspaces are made and ROS2 packages are made in each workspace.<br>
I use a follwoing terms in explanations.
Basicall each term encloses in `< >`.

* \<ws\>
  * a name of workspace
* \<package\>
  * a name of package
  * \<package\>_msgs : package for message
  * \<package\>_node : package for node
  * \<package\>_target : package for executable

# Information

1. [Making package for message](docs/making_package_for_message.md)
