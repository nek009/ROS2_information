[Back to Top page](../../README.md)

Here, a ways of making a service node is shown.
A component node is used for making a service node.
So it shuld be made by refering [making_package_of_component_node.md](making_package_of_service_node.md).

# Initial procedure
I assume a component node is already created.
And following terms are used as example.

* target packge(\<package_name\>)
  * srv_pkg
* target class(\<NODE_NAME\>)
  * SrvTest
* message used in(\<package\>_msgs/srv/\<srv file\>)
  * msg_test_msgs/srv/MsgTest.srv
    * `int a`
    * `---`
    * `int b`

## Flow of making a code for a service

**In header**

1. Include message
  * `#include "mgs_test_msgs/srv/msg_test.hpp"`
1. Declare a variable to keep a service
  * `rclcpp::Service<msg_test_msgs::srv::MsgTest>::SharedPtr srv_;`
1. Declare a callback function which shows a content of a service
  * `void handlSrv_(*1,*2);`
    * *1: const std::shared_ptr<msg_test_msgs::srv::MsgTest::Request> req
    * *2: const std::shared_ptr<msg_test_msgs::srv::MsgTest::Response> res

**In cpp**

1. Include hpp, message
1. Define a callback function
1. Register a service name and a callback function with srv_
  * `srv_ = this->create_service<msg_test_msgs::srv::MsgTest>(<service name>, a function);`

## Example of coding
Added comments `// Added below` at the point to be noticed.

```c++
#include <rclcpp/rclcpp.hpp>
// Added below
#include "msg_test_msgs/srv/msg_test.hpp"

namespace srv_pkg{

class SrvTest : public rclcpp::Node{
private:
  // Added below
  rclcpp::Service<msg_test_msgs::srv::MsgTest>::SharedPtr srv_;
  // Added below
  void handle_srv_(
    const std::shared_ptr<msg_test_msgs::srv::MsgTest::Request> request,
    const std::shared_ptr<msg_test_msgs::srv::MsgTest::Response> response
  );

public:
  SRV_PKG_PUBLIC
  SrvTestNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  SRV_PKG_PUBLIC
  SrvTestNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  SRV_PKG_PUBLIC
  ~virtual SrvTestNode();
};

} // end of namespace
```

```c++
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "srv_pkg/srv_test.hpp"
// Added below
#include "msg_test_msgs/srv/msg_test.hpp"

namespace srv_pkg{

// Added below
void SrvTestNode::handle_srv_(
  const std::shared_ptr<msg_test_msgs::srv::MsgTest::Request> request,
  const std::shared_ptr<msg_test_msgs::srv::MsgTest::Response> response
){
  // basic access to a message
  int a = request->a;
  response->b = a*2;
}

SrvTestNode::SrvTestNode(
  const rclcpp::NodeOptions& options
): SrvTestNode("",options){}

SrvTestNode::SrvTestNode(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("srv_test_node","",options){

  // Added below
  using namespace std::placeholders;
  srv_ = this->create_service<msg_test_msgs::srv::MsgTest>(
    "srv_test", // service name
    std::bind(&SrvTestNode::handle_srv_, this, _1, _2)
  );

}
...

} // end of namespace
RCLCPP_COMPONENTS_REGISTER_NODE(srv_pkg::SrvTestNode)
```
