[for Japanese:日本語](README_JP.md)

# Overview
This is information, tips or cheet sheet to make codes on ROS2 system for who basically finished [tutorial in official site(link is for foxy)](https://docs.ros.org/en/foxy/Tutorials.html).<br>
So, see [tutorial in official site(link is for foxy)](https://docs.ros.org/en/foxy/Tutorials.html) at first.
I recomend to see tutorial not only because it is official information, but also because it is easy to imagine a mechanism of ROS2, to understand basic coding technique.

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
* \<node_name\>
  * a name of node

# Information

1. [Making package for message](docs/rclcpp/making_package_for_message.md)
1. [Making package of component node](docs/rclcpp/making_package_of_component_node.md)
1. [Making package of service node](docs/rclcpp/making_package_of_service_node.md)
1. [Using own parameters](docs/rclcpp/using_own_parameters.md)
