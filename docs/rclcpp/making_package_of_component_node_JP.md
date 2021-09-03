[Topページへ](../../README_JP.md)

コンポーネントノードは普通のノードでもあり共有ライブラリでもある．
基本的には，普通のノードや共有ライブラリを作成するのではなく，コンポーネントノードを作成する．
その上で，状況に合わせて普通のノードとして使用したり使い分ける．

# 初期手順
## パッケージの作成

```shell
$ colcon_cd
$ cd <ws>
$ ros2 pkg create <package> --build-type ament_cmake --dependencies rclcpp rclcpp_components <package_of_message>_msgs --library-name <node_name>
```

通常メッセージはROS2の重要な特徴なのでコーディングで使用する．
そこで，すでにメッセージ用のパッケージを作成しており名前が確定しているなら，`--dependencies`に追加しておく．

## CMakeLists.txtの編集
関係部分のみ抜粋．<br>
target_compile_defenitionsとinstallの間に記述．

```txt
# For <library_name>
target_compile_options(<library_name>
  PUBLIC -Wall
)
rclcpp_components_register_nodes(<library_name>
  "namespace::クラス名" # イコール <package> "<package>::<LIBRARY_NAME>"
)

# For all <library_name>
ament_export_targets(
# オリジナル
#   export_${PROJECT_NAME}
# オリジナルに HAS_LIBRARY_TARGET を追加
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  # ament_target_dependenciesを見ながらexportしたいものを追加
  # 簡単な方法はament_target_dependenciesをコピー
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()
```

## ヘッダファイルの編集
### 概要
* `rclcpp::Node`クラスの継承
* 外部公開用にマクロ`<PACKAGE>_PUBLIC`の設定
* コンポーネント用と普通のノード用の使用を考え，複数のコンストラクタを用意

### 対象を継承クラスに変更
`--library-name`オプションを使用すると，作成されるクラスは`rclcpp::Node`を継承せず`rclcpp/rclcpp.hpp`をインクルードしないので継承するように変更する．

### マクロの設定
マクロ `<PACKAGE>_PUBLIC` はvisibility_control.hの中で定義されていて，windowsのために用意されている．
このマクロはヘッダファイルの中で使用し，外部に公開したいシンボル(関数などなど)に設定する．

### For constructor with namespace
コンポーネントノードのコンストラクタは引数を一つ(rclcpp::NodeOptions)しか持てない．
普通のノードとしても使用したい場合，複数のコンストラクタを用意して対応する．

### ヘッダファイルとその例
最終的にヘッダファイルは以下のようになる．

```c++
#include <rclcpp/rclcpp.hpp>

namespace \<package> {

class <LIBRARY_NAME> : rclcpp::Node{
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

パッケージ名(\<package\>)を`test_package`，ライブラリ名(\<library_name\>)を`test_pacakge_node`とした場合の例は以下のようになる．
ライブラリ名がクラス名として利用される時には，大文字にした上で`_`が削除されていることに注意する．

```c++
#include <rclcpp/rclcpp.hpp>

namespace test_package {

class TestPackageNode : rclcpp::Node{
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

## ソースコードの編集

* `#include "rclcpp_components/register_node_macro.hpp"`の追加
* `RCLCPP_COMPONENTS_REGISTER_NODE(namespace::class name)`をファイルの最後に追加
  * namespaceの外側に書くこと！

以下はパッケージ名(\<package\>)を`test_package`，ライブラリ名(\<library_name\>)を`test_pacakge_node`とした時の例である．

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
): Node("test_package_node","",options){
  ...
}
...

} // end of namespace
RCLCPP_COMPONENTS_REGISTER_NODE(test_package::TestPackageNode)
```

# パッケージ製作時の手順
## 使用するメッセージ用のパッケージを追加

**package.xml**

```xml
<package format="3">
  <build><package_of_message>_msgs</build>
```

**CMakeLists.txt**

```txt
find_package(<package_of_message>_msgs REQUIRED)

ament_target_dependencies(<library_name>
  ...
  <package_of_message>_msgs
)

ament_export_dependencies(
  ...
  <package_of_message>_msgs
)
```

## コンパイルオプションやリンカーの設定
`target_compile_options`や`target_link_libraries`を使って設定する．
以下の例では，コンパイルオプションとして`-Wall -pthread`をリンカーとして`-lpigpiod_if2`を使用する場合である．

**CMakeLists.txt**

```text
target_compile_options(<library_name>
  PUBLIC -Wall -pthread
)
target_link_libraries(<library_name>
  pigpiod_if2
)
```
