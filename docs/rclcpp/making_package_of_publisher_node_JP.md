[Topページへ](../../README_JP.md)

ここではpublisherノードを作成する方法を紹介する．
コンポーネントに追加する形で説明しているので，[making_package_of_component_node.md](making_package_of_service_node.md)を見てコンポーネントノードを作成しておくこと．

# 初期手順
以下の名前を例として用いる．

* 対象パッケージ(\<package_name\>)
  * pub_pkg
* 対象クラス(\<NODE_NAME\>)
  * PubTestNode
* 使用メッセージ(\<package\>_msgs/msg/\<srv file\>)
  * msg_test_msgs/msg/MsgTest.srv
    * `int a`
    * `int b`

## publisherノードの一般的なコーディングの流れ
publisherノードは定期的にメッセージを発信するので，一般的にTimerクラスと組み合わせて使われる．
ここでは，その場合のコーディングについて紹介する．

**In hpp files**

1. メッセージに関するヘッダファイルをインクルード
   * `#include "mgs_test_msgs/msg/msg_test.hpp"`
1. publisherを維持するための変数の宣言
   * `rclcpp::Publisher<msg_test_msgs::msg::MsgTest>::SharedPtr pub_;`
1. Timerクラスのための変数の宣言
  * `rclcpp::TimerBase::SharedPtr timer_;`

**In cpp files**

1. このクラス・使用メッセージのヘッダファイルのインクルード
1. メッセージをpublishするためのトピック名の登録
1. 定期的にpublishするためのTimerの登録

### コーディング例
追加したところに `// Added below` のコメントを追加しているので参考に．

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
  pub_ = this->create_publisher<msg_test_msgs::msg::MsgTest>("msg_test", rclcpp::QoS(KeepAll()));
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

## publisherノードに関係する部分のみのコーディングの流れ
例えばserviceの中でメッセージをpublishするなど，定期的に行わない場合，timerクラスは必要ない．
ここでは色々な使用用途を考え，publisherノードに関係する部分のみを抜き出したものを紹介する．

**In hpp files**

1. メッセージに関するヘッダファイルをインクルード
   * `#include "mgs_test_msgs/msg/msg_test.hpp"`
1. publisherを維持するための変数の宣言
   * `rclcpp::Publisher<msg_test_msgs::msg::MsgTest>::SharedPtr pub_;`

**In cpp files**

1. このクラス・使用メッセージのヘッダファイルのインクルード
1. メッセージをpublishするためのトピック名の登録

### コーディング例
追加したところに `// Added below` のコメントを追加しているので参考に．

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
  pub_ = this->create_publisher<msg_test_msgs::msg::MsgTest>("msg_test",rclcpp::QoS(KeepAll()));
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
