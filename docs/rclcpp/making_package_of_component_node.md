[Back to Top page](../../README.md)

A component node is a kind of a usual node, a shared libray.
So basically I make a component node, not a usual node or a sahred library.
And I use it as a usual node, a shared library or a component node depending on the situation.

# Initial procedure
## Package creation

```shell
$ colcon_cd
$ cd <ws>
$ ros2 pkg create <package> --build-type ament_cmake --dependencies rclcpp rclcpp_components <package_of_message>_interfaces --library-name <node_name> --license Apache-2.0
```

Usually ROS2 coding uses messages, that is a significant feature of ROS2, so \<package_of_message\>_interfaces is added at `--dependencies` if a package for messages is created and a name of it is fixed.

## Editing CMakeLists.txt
Excerpt related parts.<br>
Describe the parts between target_compile_defenitions and install.

```txt
# For <library_name>
target_compile_options(<library_name>
  PUBLIC -Wall
)
rclcpp_components_register_nodes(<library_name>
  "namespace::classname" # equal "<package>::<LIBRARY_NAME>"
)

# For all <library_name>
ament_export_dependencies(
  # add dependencies refering to ament_target_dependencies
  # easy way is to copy ament_target_dependencies
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()
```

Below is not added, is replaced with the existing setting.

```txt
ament_export_targets(
# original
#   export_${PROJECT_NAME}
# add HAS_LIBRARY_TARGET to original
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)
```

## Editing header file
### Outline
* Inherit `rclcpp::Node`
* Set macro `<PACKAGE>_PUBLIC` for all symbols you want to export.
* Prepare plural constructor for a use as component and as normal.

### Making a class a inheritance of a node class
A node created by the option `--library-name` is not a inheritance of `public rclcpp::Node` and no include file about it, `rclcpp/rclcpp.hpp`.
So you must make it inherit the class.

### Setting macro
The macro to build libraries on windows is prepared.
This macro `<PACKAGE>_PUBLIC` is written in visibility_control.h and it shuld be used for all symbols you need to export (i.e. classes or functions).

### For constructor with namespace
A constructor of a component node can take only one argument about rclcpp::NodeOptions.
So two types of constructors are prepared, one is for as a component and another is for as a usual node.

### Header file and example
Header becomes as follows.

```c++
#include <rclcpp/rclcpp.hpp>
#include "<package>/visibility_control.h"

namespace <package> {

class <LIBRARY_NAME> : public rclcpp::Node
{
public:
  <PACKAGE>_PUBLIC
  <LIBRARY_NAME>( // constructor for component
    const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
  );
  <LIBRARY_NAME>( // constructor for non-component
    const std::string& name_space,
    const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
  );
  <PACKAGE>_PUBLIC
  virtual ~<LIBRARY_NAME>(); // destructor
private:
  void hoge(); // macro is not need for non-public
};

}
```

Following is an example that \<package\>: `test_package`, \<library_name\>: `test_pacakge_node`
Notice that `_` is deleted when \<LIBRARY_NAME\> is used as a class name.

```c++
#include <rclcpp/rclcpp.hpp>
#include "test_package/visibility_control.h"

namespace test_package {

class TestPackageNode : public rclcpp::Node
{
public:
  TEST_PACKAGE_PUBLIC
  TestPackageNode( // constructor for component
    const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
  );
  TestPackageNode( // constructor for non-component
    const std::string& name_space,
    const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
  );
  TEST_PACKAGE_PUBLIC
  virtual ~TestPackageNode(); // destructor
private:
  void hoge(); // macro is not need for non-public
};

}
```

## Editing source file

* Add `#include "rclcpp_components/register_node_macro.hpp"`
* Add `RCLCPP_COMPONENTS_REGISTER_NODE(namespace::class name)` at the buttom of file
  * add the line out of the namespace

Following is an example that \<package\>: `test_package`, \<library_name\>: `test_pacakge_node`

```c++
#include <rclcpp/rclcpp.hpp>
...
#include "rclcpp_components/register_node_macro.hpp"

namespace test_package {

TestPackageNode::TestPackageNode(
  const rclcpp::NodeOptions& options
): TestPackageNode("",options){}

TestPackageNode::TestPackageNode(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("test_package_node",name_space,options){
  ...
}
...

} // end of namespace
RCLCPP_COMPONENTS_REGISTER_NODE(test_package::TestPackageNode)
```

# Procedure in production
## Adding a package for messages
### package.xml and CMakeLists.txt

**package.xml**

```xml
<package format="3">
  <build><package_of_message>_interfaces</build>
```

**CMakeLists.txt**

```text
find_package(<package_of_message>_interfaces REQUIRED)

ament_target_dependencies(<library_name>
  ...
  <package_of_message>_interfaces
)

ament_export_dependencies(
  ...
  <package_of_message>_interfaces
)
```

### Header and source files
Add include files related to messages to header or source files.
And use message as class.

### Example
Bellow is example.

* target message
  * package name
    * test_interfaces
  * type
    * msg
  * message
    * TestMsg.msg
      * `test_interfaces/msg/TestMsg.msg`

**package.xml**

```xml
<package format="3">
  <build>test_interfaces</build>
```

**CMakeLists.txt**

```text
find_package(test_interfaces REQUIRED)

ament_target_dependencies(<library_name>
  ...
  test_interfaces
)

ament_export_dependencies(
  ...
  test_interfaces
)
```

**include file**

`#include "test_interfaces/msg/test_msg.hpp"`

**class as an usage of message**

`test_interfaces::msg::TestMsg` can be used in create_publisher(or create_service if it is srv message.)

### common_interfaces
`common_interfaces` is a set of packages which contain common interface files.

See [ros2/common_interfaces on GitHub](https://github.com/ros2/common_interfaces) and search interfaces what you want to use.

Here, you want to use `common_interfaces/sensor_msgs/msg/BatteryState.msg`, editing of package.xml, CMakeLists.txt, header and source file is done based on following factors explained in `Example` section.

* target message
  * package name
    * sensor_msgs
  * type
    * msg
  * message
    * BatteryState.msg

That means `common_interfaces` isn't needed to edit these files.

## Adding compile options or linker
Use `target_compile_options` or `target_link_libraries` to add them.
Following is an exmaple for adding compile option `-pthread` and linker `-lpigpiod_if2`

**CMakeLists.txt**

```text
target_compile_options(<library_name>
  PUBLIC -Wall -pthread
)
target_link_libraries(<library_name>
  pigpiod_if2
)
```
