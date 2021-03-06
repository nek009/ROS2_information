# 概要
これはROS2に関する情報，というかROS2のプログラムのを行う上でのチートシートみたいなものである．<br>
まずはじめに，[公式のチュートリアル(for foxy)](https://docs.ros.org/en/foxy/Tutorials.html)を見ることを勧める．
その理由は，単純に公式だから，というのもあるが，ROS2の動きを理解しやすく基本的なコーディングをするための情報としてわかりやすいからである．ぜひ一読を！

## 基本的な設定と用語
以下のディレクトリをROS2のトップディレクトリとして使用する．

* ~/program/ros2

上記のディレクトリ以下にワークスペースを作成し，ワークスペースの中にROS2のパッケージを作成する．<br>
説明では以下の用語を使用する．基本的に`< >`で囲って用いる．

* \<ws\>
  * a name of workspace
* \<package\>
  * a name of package
  * \<package\>_msgs : message用
  * \<package\>_node : node用
  * \<package\>_target : 実行可能形式なもの用
* \<node_name\>
  * ノードの名前

# Information

1. [メッセージ用のパッケージ作成手順](docs/rclcpp/making_package_for_message_JP.md)
1. [コンポーネントノードのパッケージ作成手順](docs/rclcpp/making_package_of_component_node_JP.md)
1. [publisherノードの作成手順](docs/rclcpp/making_package_of_publisher_node_JP.md)
1. [serviceノードの作成手順](docs/rclcpp/making_package_of_service_node_JP.md)
1. [自身のパラメータの使用](docs/rclcpp/using_own_parameters_JP.md)
