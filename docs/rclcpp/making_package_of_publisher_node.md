[Back to Top page](../../README.md)

Here, a ways of making a publisher node is shown.
A component node is used for making a publisher node.
So it shuld be made by refering [making_package_of_component_node.md](making_package_of_service_node.md).

# Initial procedure
I assume a component node is already created.
And following terms are used as example.

* target packge(\<package_name\>)
  * pub_pkg
* target class(\<NODE_NAME\>)
  * PubTestNode
* message used in(\<package\>_msgs/msg/\<srv file\>)
  * msg_test_msgs/msg/MsgTest.srv
    * `int a`
    * `int b`

## Flow of making a standard code for a publisher
A publisher node publishes messages periodically, so this function is used with Timer class generally.

**In hpp files**

1. Include message
   * `#include "mgs_test_msgs/msg/msg_test.hpp"`
1. Declare a variable to publish messages
   * `rclcpp::Publisher<msg_test_msgs::msg::MsgTest>::SharedPtr pub_;`
1. Declare a variable for timer class
  * `rclcpp::TimerBase::SharedPtr timer_;`

**In cpp files**

1. Include hpp, message
1. Register a topic name to publish messages
1. Register a timer to publish periodically

### Example of coding
Added comments `// Added below` at the point to be noticed.

```c++
#include <rclcpp/rclcpp.hpp>
// Added below
#include "msg_test_msgs/msg/msg_test.hpp"

namespace pub_pkg{

class PubTestNode : public rclcpp::Node{
private:
  // Added below
  rclcpp::Publisher<msg_test_msgs::msg::MsgTest>::SharedPtr pub_;
  // Added below
  rclcpp::TimerBase::SharedPtr timer_;

public:
  PUB_PKG_PUBLIC
  PubTestNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  PUB_PKG_PUBLIC
  PubTestNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  PUB_PKG_PUBLIC
  ~virtual PubTestNode();
};

} // end of namespace
```

```c++
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "pub_pkg/pub_test_node.hpp"
// Added below
#include "msg_test_msgs/msg/msg_test.hpp"

namespace pub_pkg{

PubTestNode::PubTestNode(
  const rclcpp::NodeOptions& options
): PubTestNode("",options){}

PubTestNode::PubTestNode(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("srv_test_node","",options){

  // Added below
  using namespace std::chrono_literals; // for use of "s", "ms" or "ns"
  pub_ = this->create_publisher<msg_test_msgs::msg::MsgTest>("msg_test",rclcpp::QoS(KeepAll()));
  timer_ = this->create_wall_timer(
    500ms,
    [this]() ->void {
      auto msg = std::make_shared<msg_test_msgs::msg::MsgTest>();
      msg->a = 3;
      msg->b = 4;
      pub_->publish(*msg);
    }
  );

}
...

} // end of namespace
RCLCPP_COMPONENTS_REGISTER_NODE(pub_pkg::PubTestNode)
```

## Flow of making a bare bone code for a publisher
A publisher node does not need timer if it will publish in a service, or similar cases.
Here a coding related only to a publisher is introduced.

**In hpp files**

1. Include message
   * `#include "mgs_test_msgs/msg/msg_test.hpp"`
1. Declare a variable to publish messages
   * `rclcpp::Publisher<msg_test_msgs::msg::MsgTest>::SharedPtr pub_;`

**In cpp files**

1. Include hpp, message
1. Register a topic name to publish messages

### Example of coding
Added comments `// Added below` at the point to be noticed.

```c++
#include <rclcpp/rclcpp.hpp>
// Added below
#include "msg_test_msgs/msg/msg_test.hpp"

namespace pub_pkg{

class PubTestNode : public rclcpp::Node{
private:
  // Added below
  rclcpp::Publisher<msg_test_msgs::msg::MsgTest>::SharedPtr pub_;

public:
  PUB_PKG_PUBLIC
  PubTestNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  PUB_PKG_PUBLIC
  PubTestNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  PUB_PKG_PUBLIC
  ~virtual PubTestNode();
};

} // end of namespace
```

```c++
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "pub_pkg/pub_test_node.hpp"
// Added below
#include "msg_test_msgs/msg/msg_test.hpp"

namespace pub_pkg{

PubTestNode::PubTestNode(
  const rclcpp::NodeOptions& options
): PubTestNode("",options){}

PubTestNode::PubTestNode(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("srv_test_node","",options){

  // Added below
  pub_ = this->create_publisher<msg_test_msgs::msg::MsgTest>("msg_test", rclcpp::QoS(KeepAll()));
  // Added below
  // following is used in a service or similar parts usually
  auto msg = std::make_shared<msg_test_msgs::msg::MsgTest>();
  msg->a = 3;
  msg->b = 4;
  pub_->publish(*msg);

}
...

} // end of namespace
RCLCPP_COMPONENTS_REGISTER_NODE(pub_pkg::PubTestNode)
```
