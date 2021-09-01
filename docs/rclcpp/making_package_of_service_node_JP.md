[Topページへ](../../README_JP.md)

ここではserviceノードを作成する方法を紹介する．
コンポーネントに追加する形で説明しているので，[making_package_of_component_node.md](making_package_of_service_node.md)を見てコンポーネントノードを作成しておくこと．

# 初期手順
以下の名前を例として用いる．

* 対象パッケージ(\<package_name\>)
  * srv_pkg
* 対象クラス(\<NODE_NAME\>)
  * SrvTestNode
* 使用メッセージ(\<package\>_msgs/srv/\<srv file\>)
  * msg_test_msgs/srv/MsgTest.srv
    * `int a`
    * `---`
    * `int b`

## serviceノードのコーディングの流れ

**In hpp files**

1. メッセージに関するヘッダファイルをインクルード
   * `#include "mgs_test_msgs/srv/msg_test.hpp"`
1. サービスを維持するための変数の宣言
   * `rclcpp::Service<msg_test_msgs::srv::MsgTest>::SharedPtr srv_;`
1. サービスの中身となるコールバック関数を宣言
   * `void handlSrv_(*1,*2);`
     * *1: const std::shared_ptr<msg_test_msgs::srv::MsgTest::Request> req
     * *2: const std::shared_ptr<msg_test_msgs::srv::MsgTest::Response> res

**In cpp files**

1. このクラス・使用メッセージのヘッダファイルのインクルード
1. コールバック関数の実装
1. サービス名とコールバック関数の登録
   * `srv_ = this->create_service<msg_test_msgs::srv::MsgTest>(<service name>, a function);`

## コーディング例
追加したところに`// Added below`のコメントを追加しているので参考に．

```c++
#include <rclcpp/rclcpp.hpp>
// Added below
#include "msg_test_msgs/srv/msg_test.hpp"

namespace srv_pkg{

class SrvTestNode : public rclcpp::Node{
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
#include "srv_pkg/srv_test_node.hpp"
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
